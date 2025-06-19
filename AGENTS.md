# Project Agents.md Guide for OpenAI Codex

This Agents.md file provides comprehensive guidance for OpenAI Codex and other AI agents working with this codebase.

## Project Structure for OpenAI Codex Navigation

- `/examples`: Examples of usage that OpenAI Codex should maintain and extend
- `/unavlib`: Main library code that OpenAI Codex should analyze
  - `/enums`: Generated and hardcoded enums from the INAV/BF source code and MultiWii serial protocol
  - `/control`: Main user-exposed API
  - `/modules`: API functions and features implementation
  - `/tools`: User-operated scripts
- `/devtools`: User-operated tools to generate documentation and reference
- `/tests`: Test files that OpenAI Codex should maintain and extend
- `/experimental`: User-operated experimental feature development scripts