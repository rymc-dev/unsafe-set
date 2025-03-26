import pytest
from colav_unsafe_set.objects import DynamicObstacleWithMetrics
from typing import List
from colav_unsafe_set.indices_of_interest import unionise_indices_of_interest

@pytest.fixture
def obstacle1():
    # Create mock dynamic obstacle with metrics
    return DynamicObstacleWithMetrics(None, dcpa=1.0, tcpa=2.0)


@pytest.fixture
def obstacle2():
    # Create another mock dynamic obstacle with metrics
    return DynamicObstacleWithMetrics(None, dcpa=2.0, tcpa=3.0)


@pytest.fixture
def obstacle3():
    # Create another mock dynamic obstacle with metrics
    return DynamicObstacleWithMetrics(None, dcpa=3.0, tcpa=4.0)


def test_unionise_indices_of_interest_empty_lists():
    # Test the union function with empty lists
    I1 = []
    I2 = []
    I3 = []
    
    result = unionise_indices_of_interest(I1, I2, I3)
    
    # Assert that the result is an empty list
    assert result == []


def test_unionise_indices_of_interest_no_common_elements(obstacle1, obstacle2, obstacle3):
    # Test the union function with no overlapping elements
    I1 = [obstacle1]
    I2 = [obstacle2]
    I3 = [obstacle3]
    
    result = unionise_indices_of_interest(I1, I2, I3)
    
    # Assert that all obstacles are in the result
    assert len(result) == 3
    assert obstacle1 in result
    assert obstacle2 in result
    assert obstacle3 in result


def test_unionise_indices_of_interest_with_common_elements(obstacle1, obstacle2):
    # Test the union function with some overlapping elements
    I1 = [obstacle1, obstacle2]
    I2 = [obstacle2]
    I3 = [obstacle1]
    
    result = unionise_indices_of_interest(I1, I2, I3)
    
    # Assert that duplicates are removed and only unique obstacles remain
    assert len(result) == 2
    assert obstacle1 in result
    assert obstacle2 in result


def test_unionise_indices_of_interest_all_identical(obstacle1):
    # Test the union function where all lists are identical
    I1 = [obstacle1]
    I2 = [obstacle1]
    I3 = [obstacle1]
    
    result = unionise_indices_of_interest(I1, I2, I3)
    
    # Assert that only one obstacle is in the result (since duplicates are removed)
    assert len(result) == 1
    assert obstacle1 in result


def test_unionise_indices_of_interest_different_types(obstacle1, obstacle2):
    # Test the union function with different obstacles in each list
    I1 = [obstacle1]
    I2 = [obstacle2]
    I3 = []
    
    result = unionise_indices_of_interest(I1, I2, I3)
    
    # Assert that both obstacles are in the result
    assert len(result) == 2
    assert obstacle1 in result
    assert obstacle2 in result


def main():
    pytest.main()


if __name__ == '__main__':
    main()
