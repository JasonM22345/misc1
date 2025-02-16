import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import simps

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
        plt.yticks(np.arange(-1, 2, 1))  # y-axis intervals from -1 to 1
        plt.xticks(np.arange(0, 2, 1))  # x-axis intervals
        plt.xlabel("x")
        plt.ylabel("t")

        plt.subplot(3, 2, 2 * i + 2)
        plt.title(f"Average fit for ln(λ) = {ln_λ}")
        plt.plot(x, true_function, label="True function", color="green")  # h(x)
        plt.plot(x, mean_prediction, label="Mean fit", color="red")  # Average prediction
        plt.legend()
        plt.ylim(-1.5, 1.5)
        plt.yticks(np.arange(-1, 2, 1))  # y-axis intervals from -1 to 1
        plt.xticks(np.arange(0, 2, 1))  # x-axis intervals
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
    ln_λ_values = np.linspace(-2, 1.5, 50)  # Adjust range to shift intersection closer to 0.4
    squared_bias = []
    variance = []
    noise = []
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

        # Recalculate h(x): regression function h(x) = ∫t * p(t|x) dt
        regression_values = []
        for x_point in x:
            regression_sum = 0
            weight_sum = 0
            for x_data, y_data in datasets:
                weights = np.exp(-0.5 * ((x_point - x_data) / (1 / num_basis)) ** 2)
                regression_sum += np.sum(weights * y_data)
                weight_sum += np.sum(weights)
            regression_values.append(regression_sum / weight_sum)
        regression_values = np.array(regression_values)

        # Compute squared bias: (bias)² = ∫{h(x) - E_D[y(x;D)]}² p(x)dx
        bias_integrand = (regression_values - mean_prediction) ** 2
        squared_bias.append(simps(bias_integrand, x))

        # Compute variance: variance = ∫E_D[{y(x;D) - E_D[y(x;D)]}²] p(x)dx
        variance_integrand = np.var(predictions, axis=0)
        variance.append(simps(variance_integrand, x))

        # Compute noise: noise = ∫{h(x) - t}² p(x,t)dxdt
        noise_value = 0
        for x_data, y_data in datasets:
            interpolated_regression = np.interp(x_data, x, regression_values)
            noise_value += simps((interpolated_regression - y_data) ** 2, x_data)
        noise.append(noise_value / num_data_sets)

        # Simulate test error: test error = (bias)² + variance + noise
        test_error.append(squared_bias[-1] + variance[-1] + noise[-1])

    # Find minimum values and corresponding ln(λ)
    min_bias_variance = np.min(np.array(squared_bias) + np.array(variance))
    min_bias_variance_ln_lambda = ln_λ_values[np.argmin(np.array(squared_bias) + np.array(variance))]

    min_test_error = np.min(test_error)
    min_test_error_ln_lambda = ln_λ_values[np.argmin(test_error)]

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(ln_λ_values, squared_bias, label="(Bias)²", color="blue")
    plt.plot(ln_λ_values, variance, label="Variance", color="red")  # Change variance line to red
    plt.plot(ln_λ_values, np.array(squared_bias) + np.array(variance), label=f"(Bias)² + Variance (min: {min_bias_variance:.3f} at ln(λ)={min_bias_variance_ln_lambda:.2f})", color="pink")
    plt.plot(ln_λ_values, test_error, label=f"Test Error (min: {min_test_error:.3f} at ln(λ)={min_test_error_ln_lambda:.2f})", color="green")  # Solid line for test error
    plt.xlabel("ln(λ)")
    plt.ylabel("Error")
    plt.xticks(np.arange(-3, 3, 1))  # x-axis intervals
    plt.yticks(np.arange(0, 0.16, 0.03))  # y-axis intervals
    plt.ylim(bottom=0)  # Set the minimum value of the y-axis

    plt.legend()
    plt.title("Bias-Variance Trade-off")
    plt.show()

# Example usage
plot_figure_3_5()
plot_figure_3_6()
