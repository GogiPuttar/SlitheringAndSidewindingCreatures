import numpy as np

def generate_random_bitstring(string_length, r):
    """
    Generate a random bitstring of a specified length.

    Args:
    - string_length (int): The length of the bitstring.
    - r (float): The probability of getting a 1.

    Returns:
    - numpy.ndarray: The generated bitstring as a NumPy array.
    """
    bitstring = ""
    for _ in range(string_length):
        # Generate a random number between 0 and 1
        random_number = np.random.rand()
        # If the random number is less than r, set the bit to 1; otherwise, set it to 0
        bit = '1' if random_number < r else '0'
        bitstring += bit
    # Convert the bitstring to a NumPy array of integers
    return np.array(list(bitstring), dtype=int)