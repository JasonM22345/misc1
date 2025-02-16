import numpy as np
import matplotlib.pyplot as plt

# Function for Figure 3.5
def plot_figure_3_5():
    np.random.seed(0)
    
    # Parameters
    num_data_sets = 100
    num_points = 25
    num_basis = 24
    x = np.linspace(0, 1, 100)
    true_function = np.sin(2 * np.pi * x)

    # Generate data sets
    datasets = []
    for _ in range(num_data_sets):
        x_data = np.linspace(0, 1, num_points)
        noise = np.random.normal(scale=0.2, size=num_points)
        y_data = np.sin(2 * np.pi * x_data) + noise
        datasets.append((x_data, y_data))

    # Regularization parameter values
    ln_lambda_values = [2.6, -0.31, -2.4]

    plt.figure(figsize=(12, 8))

    for i, ln_lambda in enumerate(ln_lambda_values):
        lambda_value = np.exp(ln_lambda)
        predictions = []
        
        for x_data, y_data in datasets:
            # Basis functions
            phi = np.exp(-0.5 * ((x[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            phi_data = np.exp(-0.5 * ((x_data[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            
            # Compute weights with regularization
            w = np.linalg.inv(phi_data.T @ phi_data + lambda_value * np.eye(num_points)) @ phi_data.T @ y_data
            y_pred = phi @ w
            predictions.append(y_pred)

        # Average predictions
        predictions = np.array(predictions)
        mean_prediction = np.mean(predictions, axis=0)

        # Plotting
        plt.subplot(3, 2, 2 * i + 1)
        plt.title(f"Fits for ln(lambda) = {ln_lambda}")
        plt.plot(x, true_function, label="True function", color="green")
        for pred in predictions[:20]:
            plt.plot(x, pred, color="red", alpha=0.3)
        plt.ylim(-1.5, 1.5)

        plt.subplot(3, 2, 2 * i + 2)
        plt.title(f"Average fit for ln(lambda) = {ln_lambda}")
        plt.plot(x, true_function, label="True function", color="green")
        plt.plot(x, mean_prediction, label="Mean fit", color="red")
        plt.legend()
        plt.ylim(-1.5, 1.5)

    plt.tight_layout()
    plt.show()

# Function for Figure 3.6
def plot_figure_3_6():
    np.random.seed(0)

    # Parameters
    num_data_sets = 100
    num_points = 25
    num_basis = 24
    x = np.linspace(0, 1, 100)
    true_function = np.sin(2 * np.pi * x)

    # Generate data sets
    datasets = []
    for _ in range(num_data_sets):
        x_data = np.linspace(0, 1, num_points)
        noise = np.random.normal(scale=0.2, size=num_points)
        y_data = np.sin(2 * np.pi * x_data) + noise
        datasets.append((x_data, y_data))

    # Regularization parameter values
    ln_lambda_values = np.linspace(-3, 2, 50)
    squared_bias = []
    variance = []
    test_error = []

    for ln_lambda in ln_lambda_values:
        lambda_value = np.exp(ln_lambda)
        predictions = []

        for x_data, y_data in datasets:
            # Basis functions
            phi = np.exp(-0.5 * ((x[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            phi_data = np.exp(-0.5 * ((x_data[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            
            # Compute weights with regularization
            w = np.linalg.inv(phi_data.T @ phi_data + lambda_value * np.eye(num_points)) @ phi_data.T @ y_data
            y_pred = phi @ w
            predictions.append(y_pred)

        # Average predictions
        predictions = np.array(predictions)
        mean_prediction = np.mean(predictions, axis=0)
        squared_bias.append(np.mean((mean_prediction - true_function) ** 2))
        variance.append(np.mean(np.var(predictions, axis=0)))

        # Simulate test error
        test_error.append(squared_bias[-1] + variance[-1] + 0.04)  # 0.04 for noise variance

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(ln_lambda_values, squared_bias, label="(Bias)^2")
    plt.plot(ln_lambda_values, variance, label="Variance")
    plt.plot(ln_lambda_values, np.array(squared_bias) + np.array(variance), label="(Bias)^2 + Variance")
    plt.plot(ln_lambda_values, test_error, label="Test Error", linestyle="--")
    plt.xlabel("ln(lambda)")
    plt.ylabel("Error")
    plt.legend()
    plt.title("Bias-Variance Trade-off")
    plt.show()

# Example usage
plot_figure_3_5()
plot_figure_3_6()
