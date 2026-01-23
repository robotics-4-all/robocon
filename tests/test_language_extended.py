from robocon.utils import get_mm

def test_robocon_language_registration():
    mm = get_mm()
    assert mm is not None
    # Verify we can use the metamodel
    assert hasattr(mm, 'model_from_str')
