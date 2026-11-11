#!/usr/bin/env python3
"""Збирає tools/astar.c у WebAssembly і кладе його base64 у js/wasm-bin.js.
Потрібні clang із цілью wasm32 і wasm-ld:  python3 tools/build_wasm.py"""
import base64, os, subprocess, tempfile
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out = os.path.join(tempfile.mkdtemp(), 'astar.wasm')
subprocess.check_call(['clang', '--target=wasm32', '-O2', '-nostdlib', '-fno-builtin', '-ffreestanding',
    '-Wl,--no-entry', '-Wl,--export=astar', '-Wl,--export=fill_spd', '-Wl,--import-memory',
    '-Wl,--global-base=1024', '-Wl,-z,stack-size=32768', '-Wl,--initial-memory=1048576',
    '-o', out, os.path.join(root, 'tools', 'astar.c')])
b = open(out, 'rb').read()
open(os.path.join(root, 'js', 'wasm-bin.js'), 'w').write(
    '// Згенеровано tools/build_wasm.py з tools/astar.c — не редагувати руками.\n'
    f'export const WASM_B64 = "{base64.b64encode(b).decode()}";\n')
print('wasm', len(b), 'байт')
