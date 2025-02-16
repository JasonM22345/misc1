import numpy as np
import matplotlib.pyplot as plt

# Function for Figure 3.5
def plot_figure_3_5():
    np.random.seed(0)
    
    # Parameters
    num_data_sets = 100  # Number of datasets (L = 100 in textbook)
    num_points = 25  # Number of data points in each dataset (N = 25 in textbook)
    num_basis = 24  # Number of Gaussian basis functions
    x = np.linspace(0, 1, 100)  # Input range
    true_function = np.sin(2 * np.pi * x)  # True function h(x) = sin(2πx)

    # Generate synthetic data sets
    datasets = []
    for _ in range(num_data_sets):
        x_data = np.linspace(0, 1, num_points)
        noise = np.random.normal(scale=0.2, size=num_points)  # Add Gaussian noise to the data
        y_data = np.sin(2 * np.pi * x_data) + noise  # Observed data t = h(x) + noise
        datasets.append((x_data, y_data))

    # Regularization parameter values (ln(λ))
    ln_λ_values = [2.6, -0.31, -2.4]  # Different levels of regularization to illustrate bias-variance

    plt.figure(figsize=(12, 8))

    for i, ln_λ in enumerate(ln_λ_values):
        λ_value = np.exp(ln_λ)  # Convert log(λ) to λ
        predictions = []
        
        for x_data, y_data in datasets:
            # Gaussian basis functions φ(x)
            phi = np.exp(-0.5 * ((x[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            phi_data = np.exp(-0.5 * ((x_data[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            
            # Compute weights using regularized least squares: w = (ΦᵀΦ + λI)⁻¹Φᵀt
            w = np.linalg.inv(phi_data.T @ phi_data + λ_value * np.eye(num_points)) @ phi_data.T @ y_data
            y_pred = phi @ w  # Predictive function y(x) = φ(x)ᵀw
            predictions.append(y_pred)

        # Average predictions over all datasets
        predictions = np.array(predictions)
        mean_prediction = np.mean(predictions, axis=0)

        # Plotting
        plt.subplot(3, 2, 2 * i + 1)
        plt.title(f"Fits for ln(λ) = {ln_λ}")
        for pred in predictions[:20]:  # Show fits for 20 datasets
            plt.plot(x, pred, color="red", alpha=0.3)
        plt.ylim(-1.5, 1.5)
        plt.xlabel("x")
        plt.ylabel("t")

        plt.subplot(3, 2, 2 * i + 2)
        plt.title(f"Average fit for ln(λ) = {ln_λ}")
        plt.plot(x, true_function, label="True function", color="green")  # h(x)
        plt.plot(x, mean_prediction, label="Mean fit", color="red")  # Average prediction
        plt.legend()
        plt.ylim(-1.5, 1.5)
        plt.xlabel("x")
        plt.ylabel("t")

    plt.tight_layout()
    plt.show()

# Function for Figure 3.6
def plot_figure_3_6():
    np.random.seed(0)

    # Parameters
    num_data_sets = 100  # Number of datasets (L = 100 in textbook)
    num_points = 25  # Number of data points in each dataset (N = 25 in textbook)
    num_basis = 24  # Number of Gaussian basis functions
    x = np.linspace(0, 1, 100)  # Input range
    true_function = np.sin(2 * np.pi * x)  # True function h(x) = sin(2πx)

    # Generate synthetic data sets
    datasets = []
    for _ in range(num_data_sets):
        x_data = np.linspace(0, 1, num_points)
        noise = np.random.normal(scale=0.2, size=num_points)  # Add Gaussian noise to the data
        y_data = np.sin(2 * np.pi * x_data) + noise  # Observed data t = h(x) + noise
        datasets.append((x_data, y_data))

    # Regularization parameter values (ln(λ))
    ln_λ_values = np.linspace(-3, 2, 50)  # Wide range of ln(λ) values
    squared_bias = []
    variance = []
    test_error = []

    for ln_λ in ln_λ_values:
        λ_value = np.exp(ln_λ)  # Convert log(λ) to λ
        predictions = []

        for x_data, y_data in datasets:
            # Gaussian basis functions φ(x)
            phi = np.exp(-0.5 * ((x[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            phi_data = np.exp(-0.5 * ((x_data[:, None] - x_data[None, :]) / (1 / num_basis)) ** 2)
            
            # Compute weights using regularized least squares: w = (ΦᵀΦ + λI)⁻¹Φᵀt
            w = np.linalg.inv(phi_data.T @ phi_data + λ_value * np.eye(num_points)) @ phi_data.T @ y_data
            y_pred = phi @ w  # Predictive function y(x) = φ(x)ᵀw
            predictions.append(y_pred)

        # Average predictions over all datasets
        predictions = np.array(predictions)
        mean_prediction = np.mean(predictions, axis=0)

        # Compute squared bias: (bias)² = (1/N)Σ(ȳ(xₙ) - h(xₙ))²
        squared_bias.append(np.mean((mean_prediction - true_function) ** 2))

        # Compute variance: variance = (1/N)Σ(1/L)Σ(y⁽ˡ⁾(xₙ) - ȳ(xₙ))²
        variance.append(np.mean(np.var(predictions, axis=0)))

        # Simulate test error: test error = (bias)² + variance + noise
        test_error.append(squared_bias[-1] + variance[-1] + 0.04)  # Noise variance = 0.04

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(ln_λ_values, squared_bias, label="(Bias)²", color="blue")
    plt.plot(ln_λ_values, variance, label="Variance", color="orange")
    plt.plot(ln_λ_values, np.array(squared_bias) + np.array(variance), label="(Bias)² + Variance", color="pink")
    plt.plot(ln_λ_values, test_error, label="Test Error", linestyle="--", color="green")
    plt.xlabel("ln(λ)")
    plt.ylabel("Error")
    plt.legend()
    plt.title("Bias-Variance Trade-off")
    plt.show()

# Example usage
plot_figure_3_5()
plot_figure_3_6()
