import os
import matplotlib.pyplot as plt
import networkx as nx
from pgmpy.models import BayesianNetwork

def create_bayesian_network(num_particles):
    """
    Creates a Bayesian Network model where each particle is influenced by its preceding particle,
    and all particles influence the 'Drop' status.

    Args:
        num_particles (int): The number of particles in the Bayesian Network.

    Returns:
        BayesianNetwork: The constructed Bayesian Network model.
    """
    edges = []

    # Add edges where each particle is influenced by its preceding particle
    for i in range(1, num_particles):
        edges.append((f'Particle{i-1}', f'Particle{i}'))

    # Add edges where each particle influences the 'Drop' status
    for i in range(num_particles):
        edges.append((f'Particle{i}', 'Drop'))

    # Add the other specified dependencies
    edges.extend([
        ('ParticleNumber', 'Drop'),
        ('Velocity', 'Drop'),
        ('Amplitude', 'Drop'),
        ('Trajectory', 'Drop')
    ])
    
    # Create the Bayesian Network
    model = BayesianNetwork(edges)
    return model

def save_dag_diagram(num_particles, output_file='bayesian_network_dag.png'):
    """
    Generates and saves the DAG diagram for the Bayesian Network model.

    Args:
        num_particles (int): The number of particles in the Bayesian Network.
        output_file (str): The filename for the saved DAG diagram image.
    """
    # Create the Bayesian Network model
    model = create_bayesian_network(num_particles)
    
    # Convert the Bayesian Network to a NetworkX DiGraph
    graph = nx.DiGraph(model.edges())
    
    # Draw the graph using NetworkX and Matplotlib
    plt.figure(figsize=(10, 8))
    pos = nx.spring_layout(graph)  # You can change the layout as needed
    nx.draw(
        graph,
        pos,
        with_labels=True,
        node_size=2000,
        node_color='skyblue',
        font_size=10,
        font_weight='bold',
        arrows=True
    )
    
    # Save the DAG diagram to the specified file
    plt.savefig(output_file, format='png', bbox_inches='tight')
    plt.close()
    print(f"DAG diagram has been saved to '{output_file}'.")

def main():
    # Define the number of particles
    num_particles = 4
    
    # Define the output file path
    output_file = 'bayesian_network_dag.png'
    
    # Save the DAG diagram
    save_dag_diagram(num_particles, output_file)

if __name__ == "__main__":
    main()
