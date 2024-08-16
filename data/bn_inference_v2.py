import os
import pickle
import pandas as pd
import argparse
from pgmpy.models import BayesianNetwork
from pgmpy.estimators import MaximumLikelihoodEstimator
from pgmpy.inference import VariableElimination
from datetime import datetime

def create_bayesian_network(num_particles):
    # Create a chain structure for drop variables to avoid loops
    edges = []
    drop_vars = [f'Drop_{i+1}' for i in range(num_particles)]
    
    # Chain structure: Drop_1 -> Drop_2 -> Drop_3 -> ... -> Drop_n
    for i in range(num_particles - 1):
        edges.append((drop_vars[i], drop_vars[i+1]))
    
    # Additional features
    edges.extend([
        ('ParticleNumber', drop) for drop in drop_vars
    ])
    edges.extend([
        ('Velocity', drop) for drop in drop_vars
    ])
    edges.extend([
        ('Amplitude', drop) for drop in drop_vars
    ])
    edges.extend([
        ('Trajectory', drop) for drop in drop_vars
    ])
    
    model = BayesianNetwork(edges)
    return model, drop_vars

def parse_training_data(file_path):
    """
    Parse the training data from a .txt file.
    
    Args:
        file_path (str): Path to the .txt file containing the training data.

    Returns:
        list: A list of tuples representing the training data.
    """
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into components; assuming space or comma delimiter
            parts = line.strip().split()
            
            # Extract components; adjust based on your data format
            particle_number = int(parts[0])
            velocity = float(parts[1])
            amplitude = float(parts[2])
            trajectory = parts[3]
            
            # Extract drop variables (assuming the rest are drop variables)
            drop_vars = tuple(map(lambda x: x.lower() == 'true', parts[4:]))
            
            data.append((particle_number, velocity, amplitude, trajectory, *drop_vars))
    
    return data

def get_file_paths(num_particles):
    """
    Generate file paths for model and data based on the number of particles.
    
    Args:
        num_particles (int): Number of particles.

    Returns:
        tuple: (model_path, data_path)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    model_filename = f"bayesian_model_{num_particles}_particles_{timestamp}.pkl"
    data_filename = f"training_data_{num_particles}_particles_{timestamp}.pkl"
    model_path = os.path.join(os.getcwd(), model_filename)
    data_path = os.path.join(os.getcwd(), data_filename)
    return model_path, data_path

def train_or_refine_model(num_particles, data_file_path, model_path_file='model_path.txt', force_train=False):
    drop_vars = None
    
    # Get file paths for the current number of particles
    model_path, data_path = get_file_paths(num_particles)
    
    # Check if the model has been trained previously
    if not force_train and os.path.exists(model_path):
        print(f"Loading existing model from {model_path}")
        with open(model_path, 'rb') as model_file:
            model = pickle.load(model_file)
        # Extract drop_vars from the model based on num_particles
        drop_vars = [f'Drop_{i+1}' for i in range(num_particles)]
    else:
        if force_train:
            print("Force training: existing model will be overwritten.")
        else:
            print("Model does not exist. Training a new model.")
        
        # Create a new model
        model, drop_vars = create_bayesian_network(num_particles)
    
    # Parse the training data from the provided file
    data = parse_training_data(data_file_path)
    
    # Convert the data to a pandas DataFrame
    columns = ['ParticleNumber', 'Velocity', 'Amplitude', 'Trajectory'] + drop_vars
    df_data = pd.DataFrame(data, columns=columns)
    
    # Fit or refine the model using Maximum Likelihood Estimation
    model.fit(df_data, estimator=MaximumLikelihoodEstimator)
    
    # Save the model to the appropriate file
    with open(model_path, 'wb') as model_file:
        pickle.dump(model, model_file)
    print(f"Model saved to {model_path}")
    
    # Save the data used for training
    with open(data_path, 'wb') as data_file:
        pickle.dump(df_data, data_file)
    print(f"Training data saved to {data_path}")
    
    # Update the txt file with the new model path
    with open("C:/Users/weicheng/Desktop/formal_dataset/" + model_path_file.replace(".txt", "_" + str(num_particles) + ".txt"), 'w') as f:
        f.write(model_path)
        f.write("\n")
        f.write(data_path)
    
    return model, df_data

def load_model_and_data(num_particles, model_path_file='model_path.txt'):
    # Get file paths for the current number of particles
    model_path, data_path = get_file_paths(num_particles)
    file1 = open("C:/Users/weicheng/Desktop/formal_dataset/" + model_path_file.replace(".txt", "_" + str(num_particles) + ".txt"), 'r')
    lines = file1.readlines()
    model_path = lines[0][:len(lines[0])-1] # Get rid of the return symbol \n
    data_path = lines[1]
    
    if os.path.exists(model_path):
        with open(model_path, 'rb') as model_file:
            model = pickle.load(model_file)
        if os.path.exists(data_path):
            with open(data_path, 'rb') as data_file:
                df_data = pickle.load(data_file)
            return model, df_data
        else:
            raise FileNotFoundError("Training data not found.")
    else:
        raise FileNotFoundError("Model file path does not exist. Please train the model first.")

def evidence_exists(df_data, evidence):
    """
    Check if the exact evidence exists in the training data.
    """
    evidence_df = pd.DataFrame([evidence])
    # Check if there's any row in df_data that matches all the evidence
    return not df_data.merge(evidence_df, how='inner').empty

def predict(model, observation, num_particles, df_data):
    predict_pool = []
    for k in range(num_particles):
        # Prepare evidence for prediction
        evidence = {
            'ParticleNumber': k,
            'Trajectory': observation[1]
        }

        for i in range(num_particles):  # Adjusting based on the number of particles
            evidence[f'Drop_{i+1}'] = False
        
        # Check if such evidence exists in the training data
        if not evidence_exists(df_data, evidence):
            print("No prediction can be made since no matching evidence was found in the training data.")
            with open(r"C:\Users\weicheng\Desktop\ServerSampler\bin\bnTrainerInference.txt", 'w') as file:
                file.write("")
            return None
        
        # Variables to infer
        variables_to_infer = ['Velocity', 'Amplitude']
        
        # Perform inference using the loaded model
        inference = VariableElimination(model)
        query_result = inference.map_query(variables=variables_to_infer, evidence=evidence)
        
        # Combine the evidence and inferred values for the final result
        result = {**evidence, **query_result}

        predict_pool.append(result)

    # Output results
    with open(r"C:\Users\weicheng\Desktop\ServerSampler\bin\bnTrainerInference.txt", 'w') as file:
        for result in predict_pool:
            file.write(str(result['Amplitude']) + " " + str(result['Velocity']) + "\n")

    return predict_pool

def main():
    # Argument parser for command-line input
    parser = argparse.ArgumentParser(description="Train or refine a Bayesian Network model with a dynamic number of particles or use the model for prediction.")
    parser.add_argument('--num_particles', type=int, required=True, help='The number of particles in the Bayesian Network.')
    parser.add_argument('--mode', choices=['train', 'predict'], required=True, help='Mode: train the model or use it for prediction.')
    parser.add_argument('--data_file', type=str, help='Path to the training data file (txt) for training.')
    parser.add_argument('--observation', nargs=2, type=str, help='Observation data for prediction: ParticleNumber Trajectory')
    parser.add_argument('--force_train', action='store_true', help='Force training from scratch even if a model already exists.')

    args = parser.parse_args()
    
    num_particles = args.num_particles
    mode = args.mode
    
    if mode == 'train':
        if args.data_file is None:
            raise ValueError("You must provide a path to the training data file for training.")
        
        # Train or refine the model with optional force training
        model, _ = train_or_refine_model(num_particles, args.data_file, force_train=args.force_train)
    
    elif mode == 'predict':
        if args.observation is None or len(args.observation) != 2:
            raise ValueError("You must provide exactly 2 observation values for prediction: ParticleNumber Trajectory.")

        # Load the model and training data
        model, df_data = load_model_and_data(num_particles)

        # Prepare the observation
        try:
            observation = (int(args.observation[0]), args.observation[1])
        except ValueError as e:
            raise ValueError("First observation value must be an integer.") from e

        # Perform prediction
        prediction = predict(model, observation, num_particles, df_data)
        if prediction:
            print(f"Prediction result: {prediction}")

if __name__ == "__main__":
    main()
