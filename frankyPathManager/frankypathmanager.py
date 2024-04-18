import os
class FrankyPathManager:
    def __init__(self, path_file="~/franky/franky/items/Pending_items.txt", start_file="~/franky/franky/items/START.txt", finished_file="~/franky/franky/items/Finished_items.txt"):
        """
        Initialize FrankyPathManager with default file paths.

        Args:
            path_file (str): Path to the file containing pending items.
            start_file (str): Path to the file indicating start.
            finished_file (str): Path to the file containing finished items.
        """
        self.path_file = os.path.expanduser(path_file)
        self.start_file = os.path.expanduser(start_file)
        self.finished_file = os.path.expanduser(finished_file)
        self.path = []
    
    def add_endpoint(self, point):
        """
        Add an endpoint to the end of the path.

        Args:
            point (tuple): Tuple containing coordinates (x, y, z).
        """
        if self.point_is_valid(point):
            self.path.append(point)

    def add_startpoint(self, point):
        """
        Add a start point to the beginning of the path.

        Args:
            point (tuple): Tuple containing coordinates (x, y, z).
        """
        if self.point_is_valid(point):
            self.path.insert(0, point)

    def point_is_valid(self, point):
        """
        Check if the given point is valid, i.e., z-coordinate is not less than 0.02.

        Args:
            point (tuple): Tuple containing coordinates (x, y, z).

        Returns:
            bool: True if the point is valid, False otherwise.
        """
        x = point[0]
        y = point[1]
        z = point[2]

        if z<= 0.02:
            # z = 0.02 is where the gripper touches the table. It should never go lower than this
            raise ValueError("Z-coordinate should be greater than or equal to 0.02.")
        #TODO, add more validation logic
        return True

    def remove_endpoint(self, point):
        """
        Remove an endpoint from the path.

        Args:
            point (tuple): Tuple containing coordinates (x, y, z).
        """
        if point in self.path:
            self.path.remove(point)
    
    def remove_startpoint(self):
        """
        Remove the start point from the path.
        """
        if len(self.path) > 0:
            del self.path[0]
    
    def get_current_path(self):
        """
        Get the current path.

        Returns:
            list: List of tuples containing coordinates (x, y, z).
        """
        return self.path

    def write_path(self):
        """
        Write the current path to the pending items file.
        """
        with open(self.path_file, 'w') as f:
            for point in self.path:
                f.write(self.tuple_to_string(point))
        with open(self.start_file,'w') as f:
                f.write("START")
  
    def tuple_to_string(self, point):
        """
        Convert a tuple containing coordinates into the specified format.

        Args:
            point (tuple): Tuple containing coordinates (x, y, z).

        Returns:
            str: String representation of the point in the format 'x,y,z|\n'.
        """
        return "{},{},{}|\n".format(point[0], point[1], point[2])

    def string_to_tuple(self, point_str):
        """
        Convert a string in the specified format to a tuple containing coordinates.

        Args:
            point_str (str): String representation of the point in the format 'x,y,z|\n'.

        Returns:
            tuple: Tuple containing coordinates (x, y, z).
        """
        coords = point_str[:-2].split(',') #remove the "|\n"
        return float(coords[0]), float(coords[1]), float(coords[2])
 
# Example usage:
if __name__ == "__main__":
    manager = FrankyPathManager()
    example_string =['0.6,-0.2,0.02|\n',
                    '0.3,-0.2,0.02\n',
                    '0.3,0.2,0.02|\n'
                    '0.6,0.2,0.02|\n']
    for coord in example_string:
        manager.add_endpoint(coord)
    manager.write_path()
    print("Current Path:", manager.get_current_path())

