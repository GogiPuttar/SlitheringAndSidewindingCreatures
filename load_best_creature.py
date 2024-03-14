import pickle
from render_creature import render_creature

def load_best_creature(filename):
    with open(filename, 'rb') as f:
        best_creature = pickle.load(f)
    return best_creature

# Example usage
loaded_best_creature = load_best_creature('best_creature.pkl')
print("Loaded best creature:", loaded_best_creature)

render_creature(loaded_best_creature)