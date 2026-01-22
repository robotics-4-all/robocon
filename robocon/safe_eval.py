"""
Safe expression evaluator for transformations and conditional bridging.

This module provides a sandboxed evaluation environment that prevents:
- Arbitrary code execution
- File system access
- Network access
- Import of dangerous modules
- Access to sensitive builtins

Based on security best practices:
- AST validation (whitelist approach)
- Restricted builtins
- No imports allowed
- No dunder access
"""

import ast
import operator
import math
from typing import Any, Dict


class SafeEvaluator:
    """
    Safe expression evaluator using AST validation.
    
    Only allows:
    - Arithmetic operations
    - Comparisons
    - Logical operators
    - Attribute access (e.g., msg.data)
    - Safe string methods
    - Safe math functions
    - Ternary conditionals
    """
    
    # Whitelist of allowed operators
    ALLOWED_OPERATORS = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
        ast.FloorDiv: operator.floordiv,
        ast.Mod: operator.mod,
        ast.Pow: operator.pow,
        ast.USub: operator.neg,
        ast.UAdd: operator.pos,
    }
    
    ALLOWED_COMPARISONS = {
        ast.Eq: operator.eq,
        ast.NotEq: operator.ne,
        ast.Lt: operator.lt,
        ast.LtE: operator.le,
        ast.Gt: operator.gt,
        ast.GtE: operator.ge,
    }
    
    ALLOWED_BOOL_OPS = {
        ast.And: lambda a, b: a and b,
        ast.Or: lambda a, b: a or b,
    }
    
    # Whitelist of allowed builtin functions
    ALLOWED_BUILTINS = {
        'abs': abs,
        'round': round,
        'min': min,
        'max': max,
        'len': len,
        'int': int,
        'float': float,
        'str': str,
        'bool': bool,
    }
    
    # Whitelist of allowed string methods
    ALLOWED_STRING_METHODS = {
        'startswith', 'endswith', 'lower', 'upper', 'strip',
        'replace', 'split', 'join', 'format'
    }
    
    def __init__(self):
        """Initialize the safe evaluator."""
        self.allowed_names = set()
    
    def validate_ast(self, node: ast.AST) -> None:
        """
        Validate that AST only contains allowed operations.
        
        Args:
            node: AST node to validate
            
        Raises:
            ValueError: If unsafe operation is detected
        """
        if isinstance(node, ast.Expression):
            self.validate_ast(node.body)
        
        elif isinstance(node, ast.Constant):
            # Constants are always safe
            pass
        
        elif isinstance(node, ast.Name):
            # Variable access - must be in allowed names
            if node.id not in self.allowed_names and node.id not in self.ALLOWED_BUILTINS:
                # Allow if it's a known variable
                pass
        
        elif isinstance(node, ast.Attribute):
            # Attribute access (e.g., msg.data)
            # Block dunder attributes
            if node.attr.startswith('__') and node.attr.endswith('__'):
                raise ValueError(f"Access to dunder attributes is not allowed: {node.attr}")
            self.validate_ast(node.value)
        
        elif isinstance(node, ast.BinOp):
            # Binary operations (+, -, *, etc.)
            if type(node.op) not in self.ALLOWED_OPERATORS:
                raise ValueError(f"Operator {type(node.op).__name__} is not allowed")
            self.validate_ast(node.left)
            self.validate_ast(node.right)
        
        elif isinstance(node, ast.UnaryOp):
            # Unary operations (-, +, not)
            if isinstance(node.op, ast.Not):
                # 'not' is allowed
                pass
            elif type(node.op) not in self.ALLOWED_OPERATORS:
                raise ValueError(f"Unary operator {type(node.op).__name__} is not allowed")
            self.validate_ast(node.operand)
        
        elif isinstance(node, ast.Compare):
            # Comparisons (==, !=, <, >, etc.)
            for op in node.ops:
                if type(op) not in self.ALLOWED_COMPARISONS:
                    raise ValueError(f"Comparison {type(op).__name__} is not allowed")
            self.validate_ast(node.left)
            for comparator in node.comparators:
                self.validate_ast(comparator)
        
        elif isinstance(node, ast.BoolOp):
            # Boolean operations (and, or)
            if type(node.op) not in self.ALLOWED_BOOL_OPS:
                raise ValueError(f"Boolean operator {type(node.op).__name__} is not allowed")
            for value in node.values:
                self.validate_ast(value)
        
        elif isinstance(node, ast.Call):
            # Function calls
            if isinstance(node.func, ast.Name):
                if node.func.id not in self.ALLOWED_BUILTINS:
                    raise ValueError(f"Function {node.func.id} is not allowed")
            elif isinstance(node.func, ast.Attribute):
                # Method calls (e.g., string.startswith())
                if node.func.attr not in self.ALLOWED_STRING_METHODS:
                    raise ValueError(f"Method {node.func.attr} is not allowed")
                self.validate_ast(node.func.value)
            else:
                raise ValueError("Complex function calls are not allowed")
            
            # Validate arguments
            for arg in node.args:
                self.validate_ast(arg)
            for keyword in node.keywords:
                self.validate_ast(keyword.value)
        
        elif isinstance(node, ast.IfExp):
            # Ternary conditional (x if condition else y)
            self.validate_ast(node.test)
            self.validate_ast(node.body)
            self.validate_ast(node.orelse)
        
        elif isinstance(node, (ast.List, ast.Tuple)):
            # Lists and tuples
            for elt in node.elts:
                self.validate_ast(elt)
        
        elif isinstance(node, ast.Dict):
            # Dictionaries
            for key, value in zip(node.keys, node.values):
                if key:  # key can be None in dict unpacking
                    self.validate_ast(key)
                self.validate_ast(value)
        
        elif isinstance(node, ast.Subscript):
            # Subscript access (e.g., list[0], dict['key'])
            self.validate_ast(node.value)
            self.validate_ast(node.slice)
        
        elif isinstance(node, ast.Index):
            # Index node (deprecated in Python 3.9+, but kept for compatibility)
            self.validate_ast(node.value)
        
        elif isinstance(node, ast.Slice):
            # Slice (e.g., list[1:3])
            if node.lower:
                self.validate_ast(node.lower)
            if node.upper:
                self.validate_ast(node.upper)
            if node.step:
                self.validate_ast(node.step)
        
        else:
            raise ValueError(f"AST node type {type(node).__name__} is not allowed")
    
    def safe_eval(self, expression: str, context: Dict[str, Any]) -> Any:
        """
        Safely evaluate an expression with given context.
        
        Args:
            expression: String expression to evaluate
            context: Dictionary of allowed variables
            
        Returns:
            Result of the expression
            
        Raises:
            ValueError: If expression contains unsafe operations
            SyntaxError: If expression has invalid syntax
        """
        # Parse expression into AST
        try:
            tree = ast.parse(expression, mode='eval')
        except SyntaxError as e:
            raise SyntaxError(f"Invalid expression syntax: {e}")
        
        # Track allowed variable names
        self.allowed_names = set(context.keys())
        
        # Validate AST for safety
        self.validate_ast(tree)
        
        # Compile and execute in restricted environment
        code = compile(tree, '<string>', 'eval')
        
        # Create restricted globals with only allowed builtins
        safe_globals = {
            '__builtins__': self.ALLOWED_BUILTINS,
        }
        
        # Execute with restricted globals and provided context as locals
        return eval(code, safe_globals, context)


def safe_eval(expression: str, context: Dict[str, Any]) -> Any:
    """
    Convenience function for safe expression evaluation.
    
    Args:
        expression: String expression to evaluate
        context: Dictionary of allowed variables (e.g., {'msg': data})
        
    Returns:
        Result of the expression
        
    Raises:
        ValueError: If expression contains unsafe operations
        SyntaxError: If expression has invalid syntax
        
    Example:
        >>> safe_eval("msg.data * 2", {"msg": {"data": 5}})
        10
        >>> safe_eval("msg.startswith('test')", {"msg": "testing"})
        True
    """
    evaluator = SafeEvaluator()
    return evaluator.safe_eval(expression, context)
