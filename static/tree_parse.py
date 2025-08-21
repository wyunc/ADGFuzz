import re
import json
import os
import sys
import argparse

from collections import Counter
term_counter = Counter()

class TreeNode:
    def __init__(self, name, type, dependencies=None):
        self.name = name
        self.type = type  # func, top, node, leaf
        #self.dependencies = dependencies if dependencies else []
        self.dependencies = set()
        self.hasvisited = False

    def add_dependency(self, node):
        self.dependencies.add(node)

    def remove_dependency(self, node):
        # Remove dependency if it exists
        if node in self.dependencies:
            self.dependencies.remove(node)

    def set_leaf(self):
        self.type = 'leaf'

    def set_node(self):
        self.type = 'node'

    def __repr__(self):
        return f"TreeNode(name={self.name}, type={self.type}, dependencies={self.dependencies})"


def remove_circular_dependencies(function_assignments):
    # pre-delete circular_edge
    new_function_assignments = {}
    for func_name, assignments in function_assignments.items():

        passed_name = set()
        new_assignments = []

        for assignment in assignments:
            var = assignment[0]
            dependencies = assignment[1]
            passed_name.add(var)
            updated_dependencies = [dep for dep in dependencies if dep not in passed_name]
            new_assignments.append((var, updated_dependencies))
            #passed_name.add(var) #bug: self-ref, e.g. const auto &ahrs = AP::ahrs(); done: adjust this line's location

        # for var, dependencies in assignments.items():
        #
        #     updated_dependencies = [dep for dep in dependencies if dep not in passed_name]
        #     new_assignments[var] = updated_dependencies
        #     passed_name.add(var)

        new_function_assignments[func_name] = new_assignments

    return new_function_assignments

def build_tree1(function_assignments):
    # Build a tree structure from the function assignments
    tree = {}
    seen_variables = {}

    def get_node(name, node_type):
        # Create or retrieve an existing TreeNode
        if name not in seen_variables:
            seen_variables[name] = TreeNode(name, node_type)
        return seen_variables[name]



    for func_name, assignments in function_assignments.items():
        seen_variables.clear()

        func_node = TreeNode(func_name, 'func')  # The function node
        for var, dependencies in assignments:

            var_node = get_node(var, 'top')

            for dep in dependencies:
                # useless?
                dep_node = get_node(dep, 'node')

                # If dep already exists in seen_variables, check if it needs to be connected
                if dep in seen_variables:
                    # Change the dependency node to 'node' if it was previously 'top'
                    seen_variables[dep].type = 'node'
                    var_node.add_dependency(seen_variables[dep])

                # Otherwise, create a new dependency node
                else:
                    dep_node = get_node(dep, 'node')
                    var_node.add_dependency(dep_node)

            func_node.add_dependency(var_node)

        tree[func_name] = func_node

    for func_node in tree.values():
        #print(f'func_node is: {func_node}')
        def r_leaf(node):
            if not node.dependencies:
                node.set_leaf()
            for dep in node.dependencies:
                r_leaf(dep)

        r_leaf(func_node)

    return tree


def convert_tree(tree):
    """
    Only 'func', 'top', and 'leaf' type nodes are kept.
    The result is a simplified chain from 'func' -> 'top' -> 'leaf'.
    """

    def get_leaf_dependencies(node):
        leaf_nodes = []

        # If the node is of type 'leaf', add it to the result
        if node.type == 'leaf':
            leaf_nodes.append(node.name)

        elif node.type in ['top', 'node']:
            for dep in node.dependencies:
                leaf_nodes.extend(get_leaf_dependencies(dep))

        return leaf_nodes

    def count_node_dependencies(node, visited=None):
        if visited is None:
            visited = set()
        if node in visited:
            return 0
        visited.add(node)
        count = 1 if node.type == 'node' else 0
        for dep in node.dependencies:
            count += count_node_dependencies(dep, visited)
        return count

    def node_to_dict(node):
        node_dict = {}

        if node.type == 'func':
            for dep in node.dependencies:
                if dep.type == 'top':
                    leaf_nodes = get_leaf_dependencies(dep)
                    node_count = count_node_dependencies(dep)
                    if leaf_nodes:
                        node_dict[dep.name] = [leaf_nodes, node_count]

        return node_dict

    json_tree = {}
    for func_name, func_node in tree.items():
        json_tree[func_name] = node_to_dict(func_node)

    return json_tree


def save_tree_to_json(tree, filename):
    # seems not used
    json_tree = convert_tree(tree)
    with open(filename, 'w') as f:
        json.dump(json_tree, f, indent=4)

def save_to_json(data, output_file):
    with open(output_file, 'w', encoding='utf-8') as json_file:
        json.dump(data, json_file, indent=4)

def camel_to_snake(name):
    """
    Converts camelCase or PascalCase names to snake_case.
    Example: "aaBbCc" -> "aa_bb_cc", "AA_bbCc" -> "aa_bb_cc".
    """
    name = re.sub(r'([a-z])([A-Z])', r'\1_\2', name)  # Add underscore between lowercase and uppercase
    name = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)  # Add underscore between uppercase blocks and lowercase
    return name.lower()

def process_function_call_with_args_old(caller, args):
    """
    Convert 'func(var1, var2)' into 'func_var1_var2'. If no arguments, just return 'func'.
    """
    caller_snake = camel_to_snake(caller)
    if not args.strip():  # No arguments, return the function name only
        return caller_snake
    args_snake = [camel_to_snake(arg.strip()) for arg in args.split(',') if arg.strip()]
    return f"{caller_snake}_{'_'.join(args_snake)}"

def find_matching_brace(code, start_index):
    stack = []
    for i in range(start_index, len(code)):
        if code[i] == '{':
            stack.append('{')
        elif code[i] == '}':
            stack.pop()
            if not stack:
                return i
    return -1  # No matching closing brace found

def find_matching_bracket(expr, start_idx):
    stack = 0
    for i in range(start_idx, len(expr)):
        if expr[i] == '(':
            stack += 1
        elif expr[i] == ')':
            stack -= 1
            if stack == 0:
                return i
    return -1

def extract_function_calls(expr):

    function_calls = []
    pattern = re.compile(r'(\w+)\s*\(')
    pos = 0

    while pos < len(expr):
        match = pattern.search(expr, pos)
        if not match:
            break

        caller = match.group(1)
        start_idx = match.end() - 1
        end_idx = find_matching_bracket(expr, start_idx)

        if end_idx != -1:
            args = expr[start_idx + 1:end_idx]
            function_calls.append((caller, args))
            pos = end_idx + 1
        else:
            break

    return function_calls

def process_function_call_with_args(caller, args):

    # If caller is empty, that means it's a parenthesis expression, such as "(a + b)", so just drop the parenthesis
    if not caller.strip():
        return [args]

    results = []

    pos = 0
    stack = 0
    current_arg = []

    while pos < len(args):
        char = args[pos]

        if char == ',' and stack == 0:
            results.append(''.join(current_arg).strip())
            current_arg = []
        else:
            if char == '(':
                stack += 1
            elif char == ')':
                stack -= 1
            current_arg.append(char)
        pos += 1

    if current_arg:
        results.append(''.join(current_arg).strip())

    final_results = []
    for arg in results:
        nested_calls = extract_function_calls(arg)

        if nested_calls:
            for func2, inner_args in nested_calls:
                nested_results = process_function_call_with_args(func2, inner_args)
                for nested in nested_results:
                    final_results.append(f"{caller}_{nested}")
        else:
            variables = re.findall(r'\b\w+\b', arg)
            for variable in variables:
                final_results.append(f"{caller}_{variable}")

    return final_results

def replace_function_calls1(expr):

    #function_call_pattern = re.compile(r'(\w+)\s*\((.*?)\)')
    function_calls = extract_function_calls(expr)

    for caller, args in function_calls:
        processed_calls = process_function_call_with_args(caller, args)
        # expr = expr.replace(f"{caller}({args})", ", ".join(processed_calls)) #bug: omit micros(). args: '', caller: 'micros'
        if args == "":
            expr = expr.replace(f"{caller}()", caller)
        else:
            expr = expr.replace(f"{caller}({args})", ", ".join(processed_calls))

    return expr

def replace_function_calls(expr):

    function_call_pattern = re.compile(r'(\w+)\s*\((.*?)\)')

    def replace_match(match):
        caller = match.group(1)
        args = match.group(2)

        processed_calls = process_function_call_with_args(caller, args)

        # Ensure nested calls are processed correctly
        if not args.strip():
            return caller  # handle cases like func()
        else:
            return ", ".join(processed_calls)

    # Iteratively replace all function calls, even nested ones
    prev_expr = None
    while prev_expr != expr:  # Keep processing until no more changes
        prev_expr = expr
        expr = function_call_pattern.sub(replace_match, expr)

    return expr

def parse_cpp_file9(file_path):
    """
    Parses a C++ file to extract assignment statements and identify variables,
    normalizing their names to snake_case format.
    """
    function_assignments = {}

    excluded_functions = {'constrain_float', 'MAX', 'MIN'} #..

    with open(file_path, 'r', encoding='utf-8') as cpp_file:
        code = cpp_file.read()

    code = re.sub(r'^\s*//.*$', '', code, flags=re.MULTILINE)
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)

    function_pattern = re.compile(
        r"""
        (?<!\w)
        (void|bool|int|float
        |double|char|short|long
        |unsigned|signed)
        [\*\&\s]+
        (\w+::)?
        (\w+)
        \s*\([^)]*\)
        \s*(const)?
        \s*\{
        """, re.VERBOSE | re.DOTALL
    )
    assignment_pattern = re.compile(r'([\w.]+)\s*([+\-*/]?=)\s*(.+?);')

    #functions = []
    for match in function_pattern.finditer(code):
        start_index = match.start()
        end_index = find_matching_brace(code, start_index + len(match.group(0)) - 1)
        if end_index == -1:
            continue

        function_name = match.group(3)
        func_body = code[start_index:end_index + 1]

        func_body = re.sub(r"'[^']*'|\"[^\"]*\"", '', func_body)
        if function_name not in function_assignments:
            #function_assignments[function_name] = {}
            function_assignments[function_name] = []

        assignments = list(reversed(assignment_pattern.findall(func_body)))
        for var, operator, expr in assignments:

            original_var = var
            if expr == ' ' or expr == '':
                continue

            for func in excluded_functions:
                expr = re.sub(rf'\b{func}\s*\((.*?)\)', r'\1', expr)

            # if re.match(r'["\'].*["\']', expr):  # const char *failure_template = "RTL_ALT_TYPE is above-terrain but %s";
            #     continue
            # expr = re.sub(r"'[^']*'|\"[^\"]*\"", '', expr)  # delete "..." or '...'

            expr = re.sub(r'\b\d+(\.\d+)?[fFdD]?\b', '', expr)
            expr = re.sub(r'\b\w+::', '', expr)
            expr = re.sub(r'(\.|->)', '_', expr) # Replace '.' '->' to '_' 
            expr = replace_function_calls(expr)
            expr = re.sub(r'[^\w.]', ' ', expr)

            expr = re.sub(r'\b(?:b|true|false)\b', '', expr)

            expr_variables = re.findall(r'\b\w+\b', expr)
            expr_variables = [camel_to_snake(v) for v in expr_variables]

            if not expr_variables:
                continue

            function_assignments[function_name].append((original_var, expr_variables))

            # [Major Revision] static(ADG/MIS) accuracy
            # for v in expr_variables:
            #     for term in v.split('_'):
            #         if term and not term.startswith('0x'):
            #             term_counter[term] += 1


    new_function_assignments = remove_circular_dependencies(function_assignments)

    #return build_tree1(new_function_assignments)
    tree = build_tree1(new_function_assignments)
    assignments_tree = convert_tree(tree)

    for func_name in assignments_tree:
        for var in list(assignments_tree[func_name].keys()):
            assignments_tree[func_name][var] = list(assignments_tree[func_name][var])
            if not assignments_tree[func_name][var]:
                del assignments_tree[func_name][var]
    for func_name in list(assignments_tree.keys()):
        if not assignments_tree[func_name]:
            del assignments_tree[func_name]

    return assignments_tree


def process_files(input_dir, output_dir):
    for root, dirs, files in os.walk(input_dir):
        print(f"Visiting directory: {root}")
        print(f"Subdirectories: {dirs}")
        print(f"Files: {files}")
        for file in files:
            if file.endswith('.cpp'):
                cpp_file_path = os.path.join(root, file)
                #print(f"Processing file: {cpp_file_path}")

                assignments = parse_cpp_file9(cpp_file_path)

                #  # [Major Revision] static(ADG/MIS) accuracy :Delete the following five rows
                relative_path = os.path.relpath(root, input_dir) #need recovery
                output_path = os.path.join(output_dir, relative_path, f"{file.replace('.cpp', '.json')}")
                os.makedirs(os.path.dirname(output_path), exist_ok=True)
                save_to_json(assignments, output_path)
                print(f"Saved result to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="RV type and subdirs.")

    parser.add_argument(
        '--rvtype',
        required=True,
        choices=['ap', 'px4'],
        help="Type of robotic vehicle: 'ap' for ArduPilot, 'px4' for PX4"
    )
    parser.add_argument(
        '--subdirs',
        nargs='+',
        required=True,
        help="ArduPilot: ArduCopter/ArduPlane/Rover/Libraries ; PX4: lib/modules"
    )
    args = parser.parse_args()

    rvtype = args.rvtype
    subdirs = args.subdirs

    input_directory = os.getenv("ARDUPILOT_HOME")
    if input_directory is None:
        #raise Exception("ARDUPILOT_HOME environment variable is not set!")
        input_directory = '~/code/t2-ArduPilot/' # for test
    res_dir = 'inpath'

    if rvtype == 'px4':
        input_directory = os.getenv("PX4_HOME")
        input_directory = f'{input_directory}src'
        #input_directory = '~/code/PX4/src'
        res_dir = 'px4path'
        if input_directory is None:
            raise Exception("PX4_HOME environment variable is not set!")
            #input_directory = '~/code/PX4/src' # for test

    # res_dir = 'test'

    for subdir in subdirs:
        output_directory = f'{res_dir}/{subdir}/'

        full_input_path = os.path.join(input_directory, subdir)
        full_input_path = os.path.expanduser(full_input_path)  #expand '~'
        if not os.path.isdir(full_input_path):
            print(f"Directory {full_input_path} does not exist!")
            continue
        print(f"Processing directory: {full_input_path}")

        process_files(full_input_path, output_directory)

    # [Major Revision] static(ADG/MIS) accuracy
    # if term_counter:
    #     with open("px4_terms/term_countpx4.txt", "w", encoding="utf-8") as f:
    #         for term, cnt in term_counter.most_common():
    #             f.write(f"{term}:{cnt}\n")

if __name__ == "__main__":
    main()

#[ArduPilot] For test:  python tree_parse.py --rvtype ap --subdirs ArduCopter
#[ArduPilot] For complete run: python tree_parse.py --rvtype ap --subdirs ArduCopter ArduPlane Rover libraries

#[PX4] For test: python tree_parse.py --rvtype px4 --subdirs modules/airspeed_selector
#[PX4] For complete run: python tree_parse.py --rvtype px4 --subdirs lib modules


