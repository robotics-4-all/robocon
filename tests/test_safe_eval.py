import pytest
from robocon.safe_eval import safe_eval, SafeEvaluator


class TestSafeOperations:
    """Test that safe operations are allowed."""
    
    def test_basic_arithmetic(self):
        """Test basic arithmetic operations."""
        assert safe_eval("2 + 3", {}) == 5
        assert safe_eval("10 - 4", {}) ==6
        assert safe_eval("3 * 4", {}) == 12
        assert safe_eval("15 / 3", {}) == 5.0
        assert safe_eval("17 // 5", {}) == 3
        assert safe_eval("17 % 5", {}) == 2
        assert safe_eval("2 ** 3", {}) == 8
    
    def test_comparisons(self):
        """Test comparison operations."""
        assert safe_eval("5 > 3", {}) is True
        assert safe_eval("5 < 3", {}) is False
        assert safe_eval("5 >= 5", {}) is True
        assert safe_eval("5 <= 4", {}) is False
        assert safe_eval("5 == 5", {}) is True
        assert safe_eval("5 != 3", {}) is True
    
    def test_logical_operators(self):
        """Test logical operations."""
        assert safe_eval("True and False", {}) is False
        assert safe_eval("True or False", {}) is True
        assert safe_eval("not True", {}) is False
    
    def test_attribute_access(self):
        """Test safe attribute access."""
        class MockMsg:
            def __init__(self):
                self.data = 42
                self.field = type('obj', (object,), {'subfield': 'test'})()
        
        msg = MockMsg()
        assert safe_eval("msg.data", {"msg": msg}) == 42
        assert safe_eval("msg.field.subfield", {"msg": msg}) == 'test'
    
    def test_string_methods(self):
        """Test allowed string methods."""
        context = {"msg": "test_string"}
        assert safe_eval("msg.startswith('test')", context) is True
        assert safe_eval("msg.endswith('string')", context) is True
        assert safe_eval("msg.upper()", context) == "TEST_STRING"
        assert safe_eval("msg.lower()", context) == "test_string"
    
    def test_builtin_functions(self):
        """Test allowed builtin functions."""
        assert safe_eval("abs(-5)", {}) == 5
        assert safe_eval("round(3.7)", {}) == 4
        assert safe_eval("min(3, 7, 2)", {}) == 2
        assert safe_eval("max(3, 7, 2)", {}) == 7
        assert safe_eval("len('hello')", {}) == 5
    
    def test_ternary_conditional(self):
        """Test ternary conditional expressions."""
        assert safe_eval("'yes' if 5 > 3 else 'no'", {}) == 'yes'
        assert safe_eval("'yes' if 5 < 3 else 'no'", {}) == 'no'
    
    def test_complex_expression(self):
        """Test complex but safe expression."""
        class MockMsg:
            def __init__(self):
                self.data = 25.0
        
        msg = MockMsg()
        result = safe_eval("msg.data * 1.8 + 32", {"msg": msg})
        assert result == 77.0


class TestUnsafeOperations:
    """Test that unsafe operations are blocked."""
    
    def test_block_imports(self):
        """Test that imports are blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("__import__('os')", {})
    
    def test_block_file_access(self):
        """Test that file operations are blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("open('/etc/passwd')", {})
    
    def test_block_eval(self):
        """Test that eval is blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("eval('1+1')", {})
    
    def test_block_exec(self):
        """Test that exec is blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("exec('print(1)')", {})
    
    def test_block_compile(self):
        """Test that compile is blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("compile('1+1', '<string>', 'eval')", {})
    
    def test_block_dunder_access(self):
        """Test that dunder attribute access is blocked."""
        with pytest.raises(ValueError, match="dunder"):
            safe_eval("msg.__class__", {"msg": "test"})
        
        with pytest.raises(ValueError, match="dunder"):
            safe_eval("msg.__dict__", {"msg": "test"})
    
    def test_block_globals_access(self):
        """Test that globals() is blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("globals()", {})
    
    def test_block_locals_access(self):
        """Test that locals() is blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("locals()", {})
    
    def test_block_assignment(self):
        """Test that assignments are blocked (SyntaxError in eval mode)."""
        with pytest.raises(SyntaxError):
            safe_eval("x = 5", {})
    
    def test_block_function_definition(self):
        """Test that function definitions are blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("lambda x: x + 1", {})
    
    def test_block_list_comprehension(self):
        """Test that list comprehensions are blocked."""
        with pytest.raises(ValueError, match="is not allowed"):
            safe_eval("[x for x in range(10)]", {})


class TestSecurityScenarios:
    """Test real-world security scenarios."""
    
    def test_attempt_system_command(self):
        """Test blocking system command execution."""
        with pytest.raises(ValueError):
            safe_eval("__import__('subprocess').call(['ls', '-la'])", {})
    
    def test_attempt_file_read(self):
        """Test blocking file reading."""
        with pytest.raises(ValueError):
            safe_eval("open('/etc/passwd').read()", {})
    
    def test_attempt_file_write(self):
        """Test blocking file writing."""
        with pytest.raises(ValueError):
            safe_eval("open('/tmp/test', 'w').write('data')", {})
    
    def test_attempt_network_access(self):
        """Test blocking network access."""
        with pytest.raises(ValueError):
            safe_eval("__import__('urllib.request').urlopen('http://evil.com')", {})
    
    def test_attempt_code_injection(self):
        """Test blocking code injection via eval."""
        with pytest.raises(ValueError):
            safe_eval("eval(__import__('base64').b64decode('aW1wb3J0IG9z'))", {})
    
    def test_transformation_use_case(self):
        """Test valid transformation use case."""
        class DotDict(dict):
            def __getattr__(self, name):
                return self[name]
        
        data = DotDict({"data": 25.0})
        result = safe_eval("msg.data * 1.8 + 32", {"msg": data})
        assert result == 77.0
    
    def test_condition_use_case(self):
        """Test valid condition use case."""
        class DotDict(dict):
            def __getattr__(self, name):
                return self[name]
        
        data = DotDict({"data": "ERROR"})
        result = safe_eval("msg.data == 'ERROR' or msg.data == 'FATAL'", {"msg": data})
        assert result is True


class TestErrorHandling:
    """Test error handling and messages."""
    
    def test_syntax_error(self):
        """Test handling of syntax errors."""
        with pytest.raises(SyntaxError):
            safe_eval("2 +", {})
    
    def test_helpful_error_messages(self):
        """Test that error messages are helpful."""
        try:
            safe_eval("open('/etc/passwd')", {})
            assert False, "Should have raised ValueError"
        except ValueError as e:
            assert "open" in str(e) or "not allowed" in str(e)
    
    def test_undefined_variable(self):
        """Test handling of undefined variables."""
        # Should raise NameError when executed
        with pytest.raises(NameError):
            safe_eval("undefined_var", {})
