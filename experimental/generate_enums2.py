import argparse
import json
import glob
import os
import re
import enum
import keyword
import sys

import unavlib
from unavlib.enums import base_enums


# --- Patterns ---
# Enum block: Find complete `typedef enum { ... } name;` potentially spanning lines
enum_block_re = re.compile(
    r'typedef\s+enum\s*\{'      # Start
    r'(.*?)'                    # Capture content (non-greedy)
    r'\}\s*([a-zA-Z_][a-zA-Z0-9_]+)\s*;', # Closing brace, capture name, semicolon
    re.IGNORECASE | re.DOTALL   # Ignore case, '.' matches newline
)

# Enum member: Parse 'NAME [= VALUE]' within a block
# Need careful splitting/parsing within the block later
member_parse_re = re.compile(r'^\s*([a-zA-Z_][a-zA-Z0-9_]+)\s*(?:=\s*(.*))?$')

# Define: Anchored to line start, captures name and the rest of the logical line
# Excludes function-like macros. Allows empty value.
define_re = re.compile(
    r'^\s*#define\s+'              # Start of line, #define
    r'([a-zA-Z_][a-zA-Z0-9_]+)'     # Capture name
    r'(?!\()'                      # Exclude function macros
    r'(?:\s+(.*?))?'               # Optional non-capturing group for whitespace + value
                                   # Capture value (group 2) if present (non-greedy)
    r'\s*$'                        # Optional trailing whitespace, end of line
)
# --- End Patterns ---

# --- Helper Functions ---
identifier_re = re.compile(r'^[a-zA-Z_][a-zA-Z0-9_]*$')

def safe_eval_int(value_str):
    # (Keep this function as it was - used only internally by extractor)
    if not value_str: return None
    value_str = value_str.strip()
    try:
        eval_str = re.sub(r'([0-9a-fA-F]+)[uUlL]+$', r'\1', value_str, flags=re.IGNORECASE)
        if eval_str.lower().startswith('0x'): return int(eval_str, 16)
        if eval_str.isdigit() or (eval_str.startswith('-') and eval_str[1:].isdigit()):
             return int(eval_str)
    except ValueError: pass
    shift_match = re.match(r'^\(\s*1\s*<<\s*(\d+)\s*\)$', value_str)
    if shift_match:
        try: shift = int(shift_match.group(1)); return 1 << shift
        except ValueError: pass
    if re.fullmatch(r'[0-9a-fA-F]+', value_str, re.IGNORECASE) and not value_str.isdigit():
         try: return int(value_str, 16)
         except ValueError: pass
    return None

def sanitize_name(name):
    # (Keep this function as it was)
    if not isinstance(name, str): name = str(name)
    safe_name = re.sub(r'\W|^(?=\d)', '_', name)
    safe_name = safe_name.strip('_')
    if keyword.iskeyword(safe_name): safe_name += "_"
    return safe_name if safe_name else f"Unnamed_{hash(name)}"

# --- Formatter (with re.error fix, non-evaluating) ---
def format_value_expression(expr_str):
    # (Keep this function as corrected in the previous step)
    if expr_str is None: return None
    expr_str = expr_str.strip()
    if not expr_str: return None

    # --- Reject complex C constructs ---
    #if re.search(r'[\*&\->\[\]]', expr_str): return None
    if '->' in expr_str or re.search(r'[\*&\[\]]', expr_str):
        return None
    if '.' in expr_str and not re.match(r'^-?\d+\.\d+(f|F)?$', expr_str): return None

    potential_func_call = re.search(r'\b[a-zA-Z_][a-zA-Z0-9_]+\s*\(', expr_str)
    if potential_func_call:
        is_cast_expression_pattern = re.fullmatch(
            r'\s*\(\s*(?:const\s+)?(?:[a-zA-Z_]\w*)\s*\*?\s*\)\s*\(.*\)\s*',
            expr_str
        )
        if not is_cast_expression_pattern:
            return None
    # --- End check for function calls ---

    # --- Clean C Syntax ---
    cleaned_expr = re.sub(r'\(\s*(?:const\s+)?(?:[a-zA-Z_]\w*)\s*\*?\s*\)', '', expr_str).strip()
    cleaned_expr = re.sub(r'\b([0-9a-fA-F]+)[uUlL]+\b', r'\1', cleaned_expr, flags=re.IGNORECASE).strip()
    cleaned_expr = re.sub(r'\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b', r'\1', cleaned_expr, flags=re.IGNORECASE).strip()
    cleaned_expr = re.sub(r'(?<=\d)[fF]\b', '', cleaned_expr)

    if not cleaned_expr: return None

    # --- Handle Literals (No Evaluation) ---
    if cleaned_expr.startswith("'") and cleaned_expr.endswith("'") and len(cleaned_expr) == 3:
        char = cleaned_expr[1]
        if char == '"': char = '\\"'
        if char == '\\': char = '\\\\'
        return f'"{char}"'
    if cleaned_expr.startswith('"') and cleaned_expr.endswith('"'):
        return cleaned_expr

    # --- Sanitize Identifiers within the expression ---
    def sanitize_identifier(match): return sanitize_name(match.group(0))
    pythonic_expr = re.sub(r'\b([a-zA-Z_][a-zA-Z0-9_]+)\b', sanitize_identifier, cleaned_expr)

    # --- Return the Cleaned/Sanitized String ---
    if re.match(r'^[\s\w\d\(\)\|\&\^\<\>\+\-\*\/\~\.]+$', pythonic_expr):
        return pythonic_expr.strip()
    else:
        return None


# --- Preprocessor ---
def preprocess_content(content):
    """
    Cleans C content while preserving essential structure.
    1. Remove block comments.
    2. Remove line comments.
    3. Join backslash-continued lines.
    4. Remove #if/ifdef/else/elif/endif blocks (simplistic removal of lines).
    5. Remove #include, #pragma lines.
    """
    # 1. Remove block comments (non-greedy)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    # 2. Remove line comments
    content = re.sub(r'//.*', '', content)
    # 3. Join lines ending with backslash (handle optional whitespace)
    content = re.sub(r'\\\s*\n', ' ', content)

    # 4 & 5: Remove preprocessor lines we don't want
    lines = content.splitlines()
    processed_lines = []
    for line in lines:
        stripped_line = line.strip()
        if stripped_line.startswith('#'):
            if stripped_line.startswith(('#if', '#ifdef', '#ifndef', '#else', '#elif', '#endif', '#include', '#pragma', '#warning', '#error')):
                continue # Skip these directive lines
        # Keep lines that are not empty after stripping comments/directives
        if stripped_line:
             processed_lines.append(line) # Keep original spacing for enum parsing

    return "\n".join(processed_lines)


# --- Extractor (Works on Preprocessed Multi-line Content) ---
def extract_data_from_header(file_path):
    """
    Extracts enums and defines from preprocessed C header content.
    """
    enums = {}
    defines = {}
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
            raw_content = file.read()
    except IOError:
        return {}, {}

    # Preprocess the content
    content = preprocess_content(raw_content)

    # Extract Defines line by line
    lines = content.splitlines()
    for line in lines:
        define_match = define_re.match(line)
        if define_match:
            name = define_match.group(1)
            value_expr = define_match.group(2) # Group 2 is the optional value part
            # If value_expr is None (define had no value), store empty string
            value_expr_str = value_expr.strip() if value_expr is not None else ""
            safe_name = sanitize_name(name)
            if safe_name: # Ensure name is valid after sanitizing
                defines[safe_name] = (value_expr_str, None)


    # Extract Enums using block regex on the potentially multi-line content
    for enum_match in enum_block_re.finditer(content):
        enum_content_raw = enum_match.group(1).strip()
        enum_typedef_name = enum_match.group(2).strip()
        class_name = sanitize_name(enum_typedef_name)

        if not class_name or not enum_content_raw: continue

        current_enum_members = []
        value_counter = 0
        last_value_evaluated_int = None

        # Split content by comma, handle potential multi-line values inside
        # We need a more robust way than simple split if values contain commas
        # Let's try iterating with a regex to find members NAME [= VALUE] , or }
        member_find_re = re.compile(
            r'\s*([a-zA-Z_][a-zA-Z0-9_]+)\s*' # Capture Name
            r'(?:=\s*(.*?))?\s*'            # Optionally capture Value (non-greedy)
            r'(?:,|}|\Z)',                  # Stop at comma, brace, or end of string
             re.DOTALL                      # Allow '.' in value to match newline
        )

        last_pos = 0
        for member_match in member_find_re.finditer(enum_content_raw):
             # Check if we skipped characters (maybe due to comments or syntax errors)
             # This basic check might not be perfect
             if member_match.start() > last_pos + 5: # Allow some whitespace/comma gap
                  skipped_text = enum_content_raw[last_pos:member_match.start()].strip()
                  if skipped_text and skipped_text != ',':
                       print(f"  Warning: Skipped potential invalid enum content: '{skipped_text[:50]}...' in {class_name}")
                       pass # Ignore for now

             name_part = member_match.group(1)
             raw_value_expr_str = member_match.group(2) # May be None

             if not name_part: continue

             name = sanitize_name(name_part)
             output_value_str = None

             if raw_value_expr_str is not None:
                 raw_value_expr_str = raw_value_expr_str.strip()
                 # Remove trailing comma if regex captured it as part of value
                 if raw_value_expr_str.endswith(','):
                      raw_value_expr_str = raw_value_expr_str[:-1].strip()

                 if not raw_value_expr_str: continue # Skip NAME = ,

                 output_value_str = raw_value_expr_str
                 evaluated_value = safe_eval_int(raw_value_expr_str)
                 if evaluated_value is not None:
                     value_counter = evaluated_value + 1
                     last_value_evaluated_int = evaluated_value
                 else:
                     last_value_evaluated_int = None
             else: # Implicit
                 if last_value_evaluated_int is not None or not current_enum_members:
                     output_value_str = str(value_counter)
                     last_value_evaluated_int = value_counter
                     value_counter += 1
                 else:
                     output_value_str = None # Skip if prev value complex

             if name and output_value_str is not None:
                  final_value_expr_str = output_value_str.strip()
                  if final_value_expr_str:
                       current_enum_members.append((name, final_value_expr_str, None))

             last_pos = member_match.end() # Update position


        if current_enum_members:
            if any(m[0] for m in current_enum_members):
                 enums[class_name] = current_enum_members

    return enums, defines


# --- Generator (Keep As Is - Uses the NON-EVALUATING formatter) ---
def generate_python_code(all_data, base_enums_dict):
    """Generates the Python code string with Enums and defines."""
    output_lines = [
        "import enum",
        "from typing import Union, Dict, Any",
        "",
        "# Base enums", ""
    ]
    output_lines.append("# --- Base Enums ---")
    if base_enums_dict:
        for name, data in sorted(base_enums_dict.items()):
            try:
                safe_base_name = sanitize_name(name)
                # Simple representation for base data
                output_lines.append(f"{safe_base_name}: Any = {repr(data)}")
            except Exception as e:
                 output_lines.append(f"# Skipped Base Enum: {name} due to repr error: {e}")
        output_lines.append("")
    else:
        output_lines.append("# No base enums provided.")
        output_lines.append("")

    # --- Defines ---
    output_lines.append("# --- Defines Extracted from Source ---")
    consolidated_defines = {}
    all_define_names = set()
    for file_key, (enums, defines) in all_data.items():
        rel_path = file_key.replace(os.sep, '/')
        for name, (val, _) in defines.items():
            safe_name = sanitize_name(name)
            if safe_name not in consolidated_defines:
                consolidated_defines[safe_name] = (val, rel_path)
                all_define_names.add(safe_name)
    """
    # Generate Defines Output
    # This is kinda fucked NGL
    last_rel_path_def = None
    define_count = 0
    skipped_define_count = 0
    output_lines.append("\n# --- Defines ---")
    for name, (val_str, rel_path) in sorted(consolidated_defines.items()):
        fmt_val = format_value_expression(val_str) # Use NON-EVALUATING formatter

        # Add source comment ONLY if it changes
        # if rel_path != last_rel_path_def:
        #     output_lines.append(f"\n# Source: {rel_path}")
        #     last_rel_path_def = rel_path
        # OR add source comment for EVERY define
        output_lines.append(f"# Source: {rel_path}")


        if fmt_val is not None:
             if name == fmt_val:
                  output_lines.append(f"# Skipped Define: {name} = {val_str} # (Self-Referential after formatting)")
                  skipped_define_count +=1
             else:
                  output_lines.append(f"{name}: Any = {fmt_val}")
                  define_count += 1
        else:
            # Only add comment if value was not empty
            if val_str:
                output_lines.append(f"# Skipped Define: {name} = {val_str} # (Unparseable/Complex/Filtered)")
            else:
                 output_lines.append(f"# Skipped Define: {name} # (No Value)")
            skipped_define_count += 1

    print(f"Generated {define_count} unique defines, skipped {skipped_define_count}.")
    """
    # --- Enums ---
    output_lines.append("\n\n# --- Enums Extracted from Source ---")
    consolidated_enums = {}
    all_enum_member_names = set()
    for file_key, (enums, defines) in all_data.items():
        rel_path = file_key.replace(os.sep, '/')
        for name, members in enums.items():
             safe_enum_name = sanitize_name(name)
             if safe_enum_name not in consolidated_enums:
                 sanitized_members = []
                 for m_name, m_val, m_cmt in members:
                     safe_m_name = sanitize_name(m_name)
                     sanitized_members.append((safe_m_name, m_val, m_cmt))
                     all_enum_member_names.add(safe_m_name)
                 consolidated_enums[safe_enum_name] = (sanitized_members, rel_path)


    # Generate Enums Output
    last_rel_path_enum = None
    enum_count = 0
    member_count = 0
    skipped_member_count = 0
    output_lines.append("\n# --- Enums ---")
    for name, (members, rel_path) in sorted(consolidated_enums.items()):
        # Add source comment ONLY if it changes
        # if rel_path != last_rel_path_enum:
        #     output_lines.append(f"\n# Source: {rel_path}")
        #     last_rel_path_enum = rel_path
        # OR add source comment for EVERY enum
        output_lines.append(f"\n# Source: {rel_path}")


        output_lines.append(f"class {name}(enum.Enum):")
        enum_count += 1
        member_lines = []
        processed_member_names_in_enum = set()

        for m_name, m_val_str, _ in members:
            safe_m_name_base = m_name
            safe_m_name = safe_m_name_base
            suffix = 1
            while safe_m_name in processed_member_names_in_enum:
                safe_m_name = f"{safe_m_name_base}_{suffix}"
                suffix += 1

            fmt_val = format_value_expression(m_val_str)

            if fmt_val is not None:
                 if safe_m_name == fmt_val:
                      member_lines.append(f"    # Skipped Member: {safe_m_name} = {m_val_str} # (Self-Referential)")
                      skipped_member_count += 1
                 else:
                     # Ensure value isn't empty after formatting (can happen with only casts/suffixes)
                     if fmt_val.strip():
                          member_lines.append(f"    {safe_m_name} = {fmt_val}")
                          member_count += 1
                     else:
                          member_lines.append(f"    # Skipped Member: {safe_m_name} = {m_val_str} # (Empty after formatting)")
                          skipped_member_count += 1
                 processed_member_names_in_enum.add(safe_m_name)
            else:
                member_lines.append(f"    # Skipped Member: {safe_m_name} = {m_val_str} # (Unparseable/Complex/Filtered)")
                skipped_member_count += 1
                processed_member_names_in_enum.add(safe_m_name)

        if not member_lines or all('# Skipped' in line or not line.strip() for line in member_lines):
            output_lines.append("    pass # No valid members generated or all skipped")
        else:
            # Filter out empty lines that might result from skipped members
            output_lines.extend([line for line in member_lines if line.strip()])
        output_lines.append("")

    print(f"Generated {enum_count} unique enums with {member_count} members, skipped {skipped_member_count} members.")
    return "\n".join(output_lines)


# --- Main Execution Logic ---
if __name__ == '__main__':
    # (Keep main execution block exactly as it was in the previous version)
    parser = argparse.ArgumentParser(description='Generate Python Enums/Defines from C Headers (No Eval, Structure Aware)')
    parser.add_argument('--src', action='store', required=True, help='Source code directory (e.g., path/to/inav/src/main)')
    parser.add_argument('--output', action='store', default=None, help='Output Python file path. If None, uses default in unavlib/enums.')
    parser.add_argument('--dirs', action='store', default='blackbox,navigation,sensors,programming,rx,telemetry,io,flight,fc,config,msp,common', help='Comma-separated list of subdirectories to scan within src')
    parser.add_argument('--exclude-files', action='store', default='string_light.h,printf.h', help='Comma-separated list of header filenames to exclude')

    arguments = parser.parse_args()

    if not os.path.isdir(arguments.src):
        print(f"Error: Source directory not found: {arguments.src}")
        sys.exit(1)

    all_extracted_data = {}
    search_dirs = [d.strip() for d in arguments.dirs.split(',') if d.strip()]
    exclude_files = {f.strip() for f in arguments.exclude_files.split(',') if f.strip()}

    print(f"Scanning directories: {', '.join(search_dirs)}")
    print(f"Excluding files: {', '.join(exclude_files)}")
    src_base_path = os.path.abspath(arguments.src)
    files_processed = 0
    files_with_data = 0

    for d in search_dirs:
        dir_path = os.path.join(src_base_path, d)
        if not os.path.isdir(dir_path): continue
        print(f" Scanning: {d}")
        # Process only .h files generally, but could add .c if needed
        for f_path in glob.glob(os.path.join(dir_path, '*.h')):
            basename = os.path.basename(f_path)
            if basename in exclude_files: continue

            files_processed += 1
            try:
                enums, defines = extract_data_from_header(f_path) # Use NEW extractor
                if enums or defines:
                    relative_path = os.path.relpath(f_path, start=src_base_path)
                    all_extracted_data[relative_path] = (enums, defines)
                    files_with_data += 1
            except Exception as e:
                print(f"\n--- ERROR processing {f_path} ---")
                print(f"Error: {e}")
                import traceback
                traceback.print_exc()
                print("--- Continuing processing other files ---")

    print(f"\nProcessed {files_processed} header files, found data in {files_with_data}.")

    # --- Load Base Enums ---
    base_enums_output_dict = {}
    try:
        for name in dir(base_enums):
            if not name.startswith("__"):
                attr = getattr(base_enums, name)
                if isinstance(attr, (dict, list, tuple, str, int, float, bool, type(None))):
                     base_enums_output_dict[sanitize_name(name)] = attr
    except Exception as e:
         print(f"Warning: Error loading base enums: {e}")

    # --- Generate Code ---
    print("Generating Python code...")
    python_code = generate_python_code(all_extracted_data, base_enums_output_dict)

    # --- Determine Output Path ---
    try:
        with open("inav_enums_generated.py", "w", encoding='utf-8') as file:
            file.write(python_code)
        print("Done.")
    except Exception as e:
        print(f"\nError writing output file '{output_path}': {e}")
        sys.exit(1)