import re
import os
from collections import defaultdict

root = r'D:\SmartCar\RT1064_Library\Example\Coreboard_Demo\project 2.0'

h_files = []
c_files = []
for dirpath, dirnames, filenames in os.walk(root):
    rel_parts = os.path.relpath(dirpath, root).split(os.sep)
    if 'tests' in rel_parts:
        continue
    for f in filenames:
        p = os.path.join(dirpath, f)
        if f.endswith('.h'):
            h_files.append(p)
        elif f.endswith('.c'):
            c_files.append(p)

C_KEYWORDS = {'if','for','while','switch','case','return','goto','break','continue','default','do','else','sizeof','typeof','asm','__asm'}

def remove_comments(text):
    text = re.sub(r'//.*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text

def remove_strings(text):
    text = re.sub(r'"(?:\\.|[^"])*"', '""', text)
    text = re.sub(r"'(?:\\.|[^'])*'", "''", text)
    return text

def extract_func_decls(text):
    decls = {}
    text = remove_comments(text)
    lines = text.splitlines()
    merged = ''
    for line in lines:
        if line.strip().startswith('#'):
            continue
        merged += line + '\n'
    pattern = r'([\w\s\*]+?)\b(\w+)\s*\(([^)]*)\)\s*;'
    for m in re.finditer(pattern, merged, re.MULTILINE):
        ret = m.group(1).strip()
        name = m.group(2).strip()
        args = m.group(3).strip()
        if name in C_KEYWORDS or ret in C_KEYWORDS:
            continue
        if re.search(r'\b(typedef|struct|enum|union|static|extern)\b', ret):
            continue
        if '(' in ret or ')' in ret:
            continue
        if not (name[0].isalpha() or name[0] == '_'):
            continue
        if name not in decls:
            decls[name] = (ret, args)
    return decls

def extract_func_defs(text):
    defs = {}
    text = remove_comments(text)
    lines = text.splitlines()
    merged = ''
    for line in lines:
        if line.strip().startswith('#'):
            continue
        merged += line + '\n'
    pattern = r'^[ \t]*([\w\s\*]+?)\b(\w+)\s*\(([^)]*)\)\s*\{'
    for m in re.finditer(pattern, merged, re.MULTILINE):
        ret = m.group(1).strip()
        name = m.group(2).strip()
        args = m.group(3).strip()
        if name in C_KEYWORDS or ret in C_KEYWORDS:
            continue
        if re.search(r'\b(typedef|struct|enum|union)\b', ret):
            continue
        if '(' in ret or ')' in ret:
            continue
        if not (name[0].isalpha() or name[0] == '_'):
            continue
        ret_words = ret.split()
        if 'static' in ret_words:
            continue
        if name not in defs:
            defs[name] = (ret, args)
    return defs

def extract_calls(text):
    calls = set()
    text = remove_comments(text)
    text = remove_strings(text)
    lines = text.splitlines()
    for line in lines:
        if re.search(r'\b\w+\s*\([^)]*\)\s*\{', line):
            continue
        for m in re.finditer(r'(?<![\w\.\->])(\w+)[ \t]*\(', line):
            name = m.group(1)
            if name in C_KEYWORDS:
                continue
            calls.add(name)
    return calls

all_decls = {}
for h in h_files:
    with open(h, 'r', encoding='utf-8', errors='ignore') as f:
        text = f.read()
    decls = extract_func_decls(text)
    for name, (ret, args) in decls.items():
        if name not in all_decls:
            all_decls[name] = (ret, args, h)

file_defs = {}
file_calls = {}
for c in c_files:
    with open(c, 'r', encoding='utf-8', errors='ignore') as f:
        text = f.read()
    file_defs[c] = extract_func_defs(text)
    file_calls[c] = extract_calls(text)

all_defined = set()
for defs in file_defs.values():
    all_defined.update(defs.keys())

# ---- Build data ----
# For each declared function, find definition file and external callers
results = []
for name, (ret, args, h) in sorted(all_decls.items()):
    def_file = None
    for c, defs in file_defs.items():
        if name in defs:
            def_file = c
            break
    if def_file is None:
        continue  # external function
    ext_callers = []
    for c, calls in file_calls.items():
        if c == def_file:
            continue
        if name in calls:
            ext_callers.append(c)
    results.append({
        'name': name,
        'ret': ret,
        'args': args,
        'h': h,
        'c': def_file,
        'ext_callers': ext_callers,
    })

lines = []
def out(s):
    lines.append(s)

out('智能车代码接口分析报告')
out('=' * 70)
out('')
out('一、漏声明的接口（.c中定义了，但.h中没有声明）')
out('-' * 70)
count = 0
for c, defs in file_defs.items():
    c_rel = os.path.relpath(c, root)
    for name, (ret, args) in defs.items():
        if name not in all_decls:
            if 'IRQHandler' in name:
                continue
            if name in ('main', 'pit_handler', 'pit1_handler'):
                continue
            out(f'  [{count+1}] {ret} {name}({args})')
            out(f'       位置: {c_rel}')
            count += 1
if count == 0:
    out('  (无)')

out('')
out('二、已声明但无外部调用的接口（.h中声明、.c中定义，但其他.c文件未调用）')
out('   说明：以下函数虽然在.h中暴露为公共接口，但实际只在自己的.c文件内部使用，')
out('         或完全未被调用。可考虑改为static，或作为遗留代码清理。')
out('-' * 70)

# Group by module
module_groups = defaultdict(list)
for r in results:
    if len(r['ext_callers']) == 0:
        h_rel = os.path.relpath(r['h'], root)
        c_rel = os.path.relpath(r['c'], root)
        module = os.path.dirname(h_rel)
        module_groups[module].append(r)

count = 0
for module in sorted(module_groups.keys()):
    out(f'  模块: {module}')
    for r in module_groups[module]:
        out(f'    [{count+1}] {r["ret"]} {r["name"]}({r["args"]})')
        h_rel = os.path.relpath(r['h'], root)
        c_rel = os.path.relpath(r['c'], root)
        out(f'         声明: {h_rel}')
        out(f'         定义: {c_rel}')
        count += 1
if count == 0:
    out('  (无)')

out('')
out('三、声明了但在项目中无定义的函数（可能来自外部SDK/库）')
out('-' * 70)
count = 0
for name, (ret, args, h) in sorted(all_decls.items()):
    if name not in all_defined:
        h_rel = os.path.relpath(h, root)
        out(f'  [{count+1}] {ret} {name}({args});  <-- {h_rel}')
        count += 1
if count == 0:
    out('  (无)')

out('')
out('四、.c中被调用但.h中无声明且项目中无定义的函数（外部函数/宏）')
out('-' * 70)
unknown_all = set()
for c, calls in file_calls.items():
    c_rel = os.path.relpath(c, root)
    for name in calls:
        if name in all_decls or name in all_defined:
            continue
        unknown_all.add((name, c_rel))
if unknown_all:
    for name, c_rel in sorted(unknown_all):
        out(f'  - {name}()  (于 {c_rel})')
else:
    out('  (无)')

report = '\n'.join(lines)
with open(os.path.join(root, 'interface_final_report.txt'), 'w', encoding='utf-8') as f:
    f.write(report)
print('Report saved to interface_final_report.txt')
