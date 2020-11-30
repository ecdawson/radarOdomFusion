import numpy as np

def fit_with_least_squares(X, y):
    """
    Fits model for a given data using least squares.
    X should be an mxn matrix, where m is number of samples, and n is number of independent variables.
    y should be an mx1 vector of dependent variables.
    """
    #b = np.ones((X.shape[0], 1))
    #A = np.hstack((X, b))
    A = X
    theta = np.linalg.lstsq(A, y)[0]
    return theta

def evaluate_model(X, y, theta, inlier_threshold):
    """
    Evaluates model and returns total number of inliers.
    X should be an mxn matrix, where m is number of samples, and n is number of independent variables.
    y should be an mx1 vector of dependent variables.
    theta should be an (n+1)x1 vector of model parameters.
    inlier_threshold should be a scalar.
    """
    #b = np.ones((X.shape[0], 1))
    y = y.reshape((y.shape[0], 1))
    #A = np.hstack((y, X, b))
    A = np.hstack((y, X))
    
    theta = np.insert(theta, 0, -1.)
    
    distances = np.abs(np.sum(A*theta, axis=1)) / np.sqrt(np.sum(np.power(theta[:-1], 2)))
    #distances = np.abs(np.sum(A*theta, axis=1)) / np.sqrt(np.sum(np.power(theta[:], 2)))
    inliers = distances <= inlier_threshold
    num_inliers = np.count_nonzero(inliers == True)
    
    return num_inliers,distances

def ransac(X, y, fit_fn, evaluate_fn, max_iters=200, samples_to_fit=2, inlier_threshold=0.1, min_inliers=5):
    best_model = None
    best_model_performance = 0
    
    num_samples = X.shape[0]
    
    for i in range(max_iters):
        sample = np.random.choice(num_samples, size=samples_to_fit, replace=False)
        model_params = fit_fn(X[sample], y[sample])
        model_performance,_ = evaluate_fn(X, y, model_params, inlier_threshold)
        if model_performance < min_inliers:
            continue
        
        if model_performance > best_model_performance:
            best_model = model_params
            best_model_performance = model_performance
    
    return best_model,model_performance
