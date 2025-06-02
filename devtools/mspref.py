import re
def generate_msp_dict(markdown_content):
    msp_references = {}

    bin_type_map = {
        "enum": "B",
        "uint8_t": "B",
        "uint16_t": "H",
        "uint32_t": "I",
        "uint64_t": "Q",
        "int8_t": "b",
        "int16_t": "h",
        "int32_t": "i",
        "int64_t": "q",
        "float": "f",
        "double": "d",
        "char": "c",
        "bool": "?",
        "boolean": "?",
        "boxBitmask_t": "Q", # Assuming uint64_t based on common usage
    }

    message_blocks = re.split(r'\n---\n', markdown_content)

    for block_idx, block in enumerate(message_blocks):
        # Skip blocks that are not message definition sections (like intro or potentially footers)
        if not block.strip().startswith("## MSPv1") and \
           not block.strip().startswith("## MSPv2"):
            if block_idx == 0:  # Specifically skip the intro block
                continue
            # If there are other non-message blocks after intro, this might need adjustment
            # but for now, assume only intro needs this special skip.

        # Regex for message definition lines: ### `MSG_NAME` (ID_INFO_STR)
        # Group 1: Message Name
        # Group 2: ID Info String (content within parentheses)
        # Group 3: Message body content
        message_section_regex = r'### `(MSP[A-Za-z0-9_]+(?:_EX|_EXT2)?)`\s*\(([^)]+)\)([\s\S]*?)(?=\n### `|\Z)'
        
        for msg_match in re.finditer(message_section_regex, block):
            msg_name = msg_match.group(1).strip()
            id_info_str = msg_match.group(2).strip()
            msg_content = msg_match.group(3)

            msg_code = None
            # Try to parse ID info string for decimal code
            # Pattern 1: DEC / HEX ... (e.g., "1 / 0x01")
            match_dec_first = re.match(r"(\d+)\s*/\s*0x[\dA-Fa-f]+", id_info_str)
            if match_dec_first:
                msg_code = int(match_dec_first.group(1))
            else:
                # Pattern 2: HEX / DEC ... (e.g., "0x1001 / 4097")
                match_hex_first = re.match(r"0x[\dA-Fa-f]+\s*/\s*(\d+)", id_info_str)
                if match_hex_first:
                    msg_code = int(match_hex_first.group(1))
            
            if msg_code is None:
                # Fallback for cases like (255 / 0xFF) if the above don't catch due to spacing or other variations
                # This is a more general attempt if the structured ones fail
                parts = [p.strip() for p in id_info_str.split('/')]
                if len(parts) >= 1:
                    if parts[0].isdigit(): # If first part is decimal
                        msg_code = int(parts[0])
                    elif len(parts) >= 2 and parts[1].isdigit(): # If second part is decimal
                        msg_code = int(parts[1])

            if msg_code is None:
                print(f"Warning: Could not parse ID for message {msg_name} from ID_INFO '{id_info_str}'. Skipping.")
                continue

            direction_str_match = re.search(r'\*\*Direction:\*\* (In/Out|Out|In|N/A(?: \(Indicator\))?)', msg_content)
            if not direction_str_match:
                # print(f"Warning: No direction found for {msg_name}. Skipping.") # Potentially skip or default
                continue
            direction_str = direction_str_match.group(1).strip()

            direction_val = -1
            if direction_str == "Out":
                direction_val = 1
            elif direction_str == "In":
                direction_val = 0
            elif direction_str == "In/Out":
                direction_val = 2
            elif direction_str.startswith("N/A"):
                direction_val = -1 # Explicitly for N/A

            payload_fields = []
            struct_parts = []
            
            payload_table_content_area = msg_content # Start with the whole message content
            
            # Determine which payload section to parse based on direction
            if direction_str == "In/Out":
                # For In/Out, default to Reply Payload for struct, but Request might be used by some GCS for sending
                # The prompt asks for 'struct' which usually refers to data received (Reply) or data sent (Request)
                # Let's prioritize Reply for struct representation if available
                reply_payload_match = re.search(r'\*\*Reply Payload(?: \(Format \d+\))?:\*\*([\s\S]*?)(?=\n\*\*(?:Request Payload|Notes|Field Tables for other formats):\*\*|\n### `|\Z)', msg_content)
                if reply_payload_match:
                    payload_table_content_area = reply_payload_match.group(1)
                else: # If no explicit Reply, check Request
                    request_payload_match = re.search(r'\*\*Request Payload(?: \(Format \d+\))?:\*\*([\s\S]*?)(?=\n\*\*(?:Reply Payload|Notes|Field Tables for other formats):\*\*|\n### `|\Z)', msg_content)
                    if request_payload_match:
                         payload_table_content_area = request_payload_match.group(1)
                    # If neither, it might be a simple payload described directly or "Payload: None"
            elif direction_str == "In":
                request_payload_match = re.search(r'\*\*Request Payload(?: \(Format \d+\))?:\*\*([\s\S]*?)(?=\n\*\*(?:Notes|Field Tables for other formats):\*\*|\n### `|\Z)', msg_content)
                if request_payload_match:
                    payload_table_content_area = request_payload_match.group(1)
            # For "Out", the general "Payload:" section is usually the one.

            repetition_factor_str = None
            repetition_match = re.search(r'Payload(?: \(Matches `.*?` structure\))?:\s*Repeated\s+(?:`([\w_]+)`|([\d]+))\s+times:', payload_table_content_area)
            if repetition_match:
                repetition_factor_str = repetition_match.group(1) if repetition_match.group(1) else repetition_match.group(2)
                # The actual table follows this line
                payload_table_content_area = payload_table_content_area.split(repetition_match.group(0), 1)[-1]

            # General payload section search if not already specific
            if not (("Request Payload:" in payload_table_content_area and (direction_str == "In" or direction_str == "In/Out")) or \
                    ("Reply Payload:" in payload_table_content_area and direction_str == "In/Out")):
                general_payload_match = re.search(r'\*\*Payload(?: \(Matches `.*?` structure\))?(?: \(Format \d+\))?:\*\*([\s\S]*?)(?=\n\*\*(?:Notes|Field Tables for other formats):\*\*|\n### `|\Z)', payload_table_content_area, re.IGNORECASE)
                if general_payload_match:
                    payload_table_content_area = general_payload_match.group(1)
                elif "Payload: None" in payload_table_content_area:
                    payload_table_content_area = "" # No fields

            if "Payload: None" in payload_table_content_area:
                 pass 
            else:
                # Regex for payload table rows: | Field | C Type | Size (Bytes) | Optional Units | Optional Description |
                table_row_regex = r'\|\s*`?([\w\s\(\)\+\-\*/\.:<>\[\]\{\}#&;]+)`?\s*\|\s*`?([\w\s_\[\]\(\)\*:]+)`?\s*\|\s*([\w\s\.\*\[\]\(\)]+(?:\s*Variable)?)\s*\|(?:[^\|\n]*\|)?(?:[^\|\n]*\|)?'
                
                for row_match in re.finditer(table_row_regex, payload_table_content_area):
                    field_name = row_match.group(1).strip().replace('`', '')
                    if field_name.lower() == "field" or field_name.lower().startswith("payload (format"): # Skip table header
                        continue
                    if "**vehicle data (repeated" in field_name.lower() or \
                       "**parts data (repeated" in field_name.lower() or \
                       "**visibility data:**" in field_name.lower() or \
                       "**text data:**" in field_name.lower(): # Skip sub-headers within tables
                        continue


                    c_type = row_match.group(2).strip().replace('`', '')
                    size_bytes_str = row_match.group(3).strip().replace('`', '')

                    payload_fields.append([field_name, c_type])
                    
                    normalized_c_type_for_map = "bool" if c_type.lower() == "boolean" else c_type
                    
                    array_match_re = re.match(r'([\w\s_:]+)\s*\[\s*([\w\d_]*)\s*\]', normalized_c_type_for_map)
                    if array_match_re:
                        base_type = array_match_re.group(1).strip()
                        count_str_array = array_match_re.group(2).strip()

                        if base_type == "char":
                            if count_str_array.isdigit():
                                struct_parts.append(f"{count_str_array}s")
                            elif count_str_array: # Symbolic (e.g., MAX_NAME_LENGTH)
                                struct_parts.append(f"{count_str_array}s") # Represent as symbolic string length
                            else: # char[] (variable)
                                struct_parts.append("s") # 's' can imply variable length bytes handled by payload size
                        else:
                            fmt_char = bin_type_map.get(base_type)
                            if fmt_char:
                                if count_str_array.isdigit():
                                    struct_parts.append(fmt_char * int(count_str_array))
                                elif count_str_array: # Symbolic
                                    struct_parts.append(f"{count_str_array}{fmt_char}")
                                else: # type[] (variable like uint8_t[])
                                    if "Variable" in size_bytes_str or "variable" in size_bytes_str:
                                        struct_parts.append("s") # Treat as variable bytes
                                    else: # Array of known type but unknown fixed size from string
                                         struct_parts.append(f"<?>[]") # Placeholder for array of known type, unknown count
                            else: # Array of unknown/complex type
                                if count_str_array.isdigit(): struct_parts.append(f"{count_str_array}*<{base_type}>")
                                elif count_str_array: struct_parts.append(f"{count_str_array}*<{base_type}>")
                                else: struct_parts.append(f"<{base_type}>[]")
                    else:
                        fmt_char = bin_type_map.get(normalized_c_type_for_map)
                        if fmt_char:
                            struct_parts.append(fmt_char)
                        elif normalized_c_type_for_map.endswith("_t") or "struct" in normalized_c_type_for_map.lower():
                            struct_parts.append(f"<{normalized_c_type_for_map}>")
                        elif "Variable" in size_bytes_str or "variable" in size_bytes_str: # Variable size non-array
                             struct_parts.append("s")
                        else:
                            struct_parts.append("<?>")

            final_struct_str = "".join(struct_parts)
            if repetition_factor_str:
                if repetition_factor_str.isdigit():
                    final_struct_str = final_struct_str * int(repetition_factor_str)
                else: # Symbolic repetition (e.g., MAX_MOTORS)
                    final_struct_str = f"{repetition_factor_str}*({final_struct_str})" if final_struct_str else ""

            total_bytes = 0
            if final_struct_str: # Only attempt to calculate if a struct string was formed
                # This is a simplified byte count.
                # It assumes symbolic constants (like MAX_MOTORS) are resolvable to numbers
                # or that placeholders <?> are handled/ignored appropriately by the user.
                # It won't be perfect for struct strings with unresolved symbolic parts
                # or nested structs without their sizes defined here.

                # Little-endian '<', big-endian '>', native '=' don't count towards bytes
                effective_struct_str = final_struct_str.lstrip('<>=!@')

                # Handle symbolic repetitions like "MAX_SUPPORTED_MOTORS*(BH)"
                # This is a very basic heuristic and won't solve complex algebraic expressions.
                # For accurate byte counts with symbolic parts, those symbols need to be resolved
                # to their numeric values *before* this calculation.
                # We'll make a best effort for simple "N*(...)" patterns.

                # First, resolve simple N*char patterns like 16B, 4s, etc.
                # Then handle N*(...) patterns
                
                # This is a placeholder for a more complex symbolic evaluation if needed.
                # For now, we'll sum up known type sizes and basic repetitions.
                
                # Create a temporary string to expand simple numeric repetitions
                expanded_struct = ""
                i = 0
                while i < len(effective_struct_str):
                    # Check for numeric prefix for basic types
                    num_prefix_match = re.match(r"(\d+)([A-Za-z?])", effective_struct_str[i:])
                    if num_prefix_match:
                        count = int(num_prefix_match.group(1))
                        char = num_prefix_match.group(2)
                        expanded_struct += char * count
                        i += len(num_prefix_match.group(0))
                        continue
                    
                    # Check for symbolic_N*char patterns like MAX_FOO*B - these are hard to calc here
                    # We'll just take the char for now and assume user resolves MAX_FOO
                    symbolic_prefix_char_match = re.match(r"([A-Z_0-9]+)\*([A-Za-z?])", effective_struct_str[i:])
                    if symbolic_prefix_char_match:
                        # Cannot determine bytes for symbolic multiplier here accurately
                        # We could add a special marker or try to estimate if MAX_... is in a known list
                        # For now, just add the single char's size and the user must be aware
                        # Or, we mark total_bytes as None or "Symbolic"
                        char = symbolic_prefix_char_match.group(2)
                        expanded_struct += char
                        i += len(symbolic_prefix_char_match.group(0))
                        # To signal it's symbolic, you might set total_bytes = "Symbolic" later
                        continue

                    # Check for symbolic_N*(...) patterns
                    symbolic_block_match = re.match(r"([A-Z_0-9]+)\*\(([^)]+)\)", effective_struct_str[i:])
                    if symbolic_block_match:
                        # Cannot determine bytes for symbolic multiplier here accurately
                        # We could add a special marker or try to estimate
                        # For now, just add the inner block's chars and the user must be aware
                        inner_block = symbolic_block_match.group(2)
                        expanded_struct += inner_block # Add inner content, multiplier is symbolic
                        i += len(symbolic_block_match.group(0))
                        continue
                    
                    # Handle <struct_name> placeholders
                    struct_placeholder_match = re.match(r"(<[^>]+>)", effective_struct_str[i:])
                    if struct_placeholder_match:
                        # Cannot determine bytes for unknown struct placeholders here
                        # Could set total_bytes to "Symbolic" or None
                        # For now, we skip adding bytes for these.
                        i += len(struct_placeholder_match.group(0))
                        continue
                    
                    # Add non-prefixed character
                    expanded_struct += effective_struct_str[i]
                    i += 1
                
                for char_code in expanded_struct:
                    if char_code in ['x']: # padding byte, counts as 1
                        total_bytes += 1
                    elif char_code in ['c', 'b', 'B', '?']: # 1 byte
                        total_bytes += 1
                    elif char_code in ['h', 'H']: # 2 bytes
                        total_bytes += 2
                    elif char_code in ['i', 'I', 'f']: # 4 bytes
                        total_bytes += 4
                    elif char_code in ['q', 'Q', 'd']: # 8 bytes
                        total_bytes += 8
                    elif char_code == 's':
                        # 's' without a pre-count in the *final* struct string is problematic
                        # for fixed byte counting unless it's the *only* element or last,
                        # implying "rest of the data". If it came from char[N]s, Ns, it's fine.
                        # If it came from char[] (variable), then this byte count is not fixed.
                        # This simplified counter will assume 's' derived from char[N]s has already been expanded.
                        # If 's' remains from a truly variable field, total_bytes becomes an estimate or "Variable".
                        # For now, if 's' is in expanded_struct, it implies 1 byte from a non-prefixed 's'
                        # or it's part of a "Ns" which would have been expanded.
                        # This part is tricky without full context of how 's' was derived.
                        # A single 's' usually means a string of bytes, count is separate.
                        # For char[N] -> Ns, the N * 1 byte for each char of 's' is what we want.
                        # The current expansion logic should turn "4s" into "ssss", so this will count 4.
                        total_bytes += 1 # Count each 's' char from expansion as 1 byte
                    elif char_code in ['P']: # void * (pointer) - size is platform dependent, usually 4 or 8.
                                            # Not typically used directly in MSP wire formats this way.
                                            # Assuming 4 for a common case if it ever appeared.
                        total_bytes += 4 # Placeholder, unlikely
                    # '<', '>', '=', '!' are for endianness/alignment, not data bytes
                    # '(', ')' are for grouping with symbolic multipliers, handled above.
                    # '*' is part of symbolic multipliers, handled above.
                    # if char_code not in '<>=!@()*':
                    #    print(f"Warning: Unknown char_code '{char_code}' in byte count for {msg_name}")

            msp_references[msg_name] = {
                "code": msg_code,
                "direction": direction_val,
                "struct": f"<{final_struct_str}" if final_struct_str else "<", # Add endianness if struct exists, ensure "<" for empty
                "byte_count": total_bytes if final_struct_str else 0, # Add the calculated byte count
                "payload": payload_fields
            }
            if not final_struct_str and payload_fields: # If struct couldn't be formed but payload exists
                 msp_references[msg_name]["struct"] = "<?" # Indicate struct is unknown but payload exists
                 msp_references[msg_name]["byte_count"] = -1

    return msp_references

import json
with open("msp_messages_reference.md","r") as file:
    f = file.read()
d = generate_msp_dict(f)
j = json.dumps(d, indent=4)
with open("msp_messages_ref.py","w+") as file:
    file.write(j)

# The markdown_content variable should be populated with the actual markdown text provided in the prompt.
# For brevity in this example, I'll use a placeholder.
# To run this, replace `markdown_text_from_prompt` with the full markdown.
# markdown_text_from_prompt = """... (Full Markdown Text Here) ..."""
# msp_data_dict = generate_msp_dict(markdown_text_from_prompt)
# import json
# print(json.dumps(msp_data_dict, indent=4))
