import unittest
from unittest.mock import patch
import os
from frankypathmanager import FrankyPathManager  # Assuming the class is in a file named path_manager.py

class TestFrankyPathManager(unittest.TestCase):
    def setUp(self):
        self.manager = FrankyPathManager()
        # Create temporary files for testing
        self.temp_path_file = "test_pending_items.txt"
        self.temp_start_file = "test_start.txt"
        self.manager.path_file = self.temp_path_file
        self.manager.start_file = self.temp_start_file
    def tearDown(self):
        # Check if temporary files exist before attempting to remove them
        if os.path.exists(self.temp_path_file):
            os.remove(self.temp_path_file)
        if os.path.exists(self.temp_start_file):
            os.remove(self.temp_start_file)

    def test_add_endpoint(self):
        # Test that adding an endpoint with z-coordinate less than 0.02 raises ValueError
        with self.assertRaises(ValueError):
            self.manager.add_endpoint((1, 2, 0.01))
        
        # Test that adding a valid endpoint works correctly
        self.manager.add_endpoint((1, 2, 3))
        self.assertIn((1, 2, 3), self.manager.path)

    def test_add_startpoint(self):
        # Test that adding a startpoint with z-coordinate less than 0.02 raises ValueError
        with self.assertRaises(ValueError):
            self.manager.add_startpoint((1, 2, 0.01))

        # Test that adding a valid startpoint works correctly
        self.manager.add_startpoint((1, 2, 3))
        self.assertIn((1, 2, 3), self.manager.path)

    def test_remove_endpoint(self):
        # Test that removing an endpoint works correctly
        self.manager.path = [(1, 2, 3), (4, 5, 6)]
        self.manager.remove_endpoint((1, 2, 3))
        self.assertNotIn((1, 2, 3), self.manager.path)

    def test_remove_startpoint(self):
        # Test that removing a startpoint works correctly
        self.manager.path = [(1, 2, 3), (4, 5, 6)]
        self.manager.remove_startpoint()
        self.assertNotIn((1, 2, 3), self.manager.path)

    def test_point_is_valid(self):
        # Test that point_is_valid correctly validates the z-coordinate
        with self.assertRaises(ValueError):
            self.manager.point_is_valid((1, 2, 0.01))

    def test_write_path(self):
        # Set the path attribute
        self.manager.path = [(1, 2, 3), (4, 5, 6)]
        # Write path to files
        self.manager.write_path()
        # Read content of the temporary path file
        with open(self.temp_path_file, 'r') as f:
            path_content = f.read()
        # Read content of the temporary start file
        with open(self.temp_start_file, 'r') as f:
            start_content = f.read()
        # Expected content
        expected_path_content = "1,2,3|\n4,5,6|\n"
        expected_start_content = "START"
        # Check if content matches
        self.assertEqual(path_content, expected_path_content)
        self.assertEqual(start_content, expected_start_content)

    def test_string_to_tuple(self):
        # Test that string_to_tuple correctly converts string to tuple
        result = self.manager.string_to_tuple("1,2,3|\n")
        self.assertEqual(result, (1.0, 2.0, 3.0))

    def test_tuple_to_string(self):
        # Test that tuple_to_string correctly converts tuple to string
        result = self.manager.tuple_to_string((1, 2, 3))
        self.assertEqual(result, "1,2,3|\n")

if __name__ == '__main__':
    unittest.main()

