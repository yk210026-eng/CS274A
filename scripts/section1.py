#!/usr/bin/env python3
# add import and helper functions here
import numpy as np

if __name__ == "__main__":
    print("hello world")
    
    np.random.seed(42)
    A = np.random.normal(size=(4, 4))
    B = np.random.normal(size=(4, 2))
    print(A@B)
    
    np.random.seed(42)
    x = np.random.normal(size=(4, 10))
    G = x @ x.T
    s = np.diag(G)
    print(s[None, :] + s[:, None] - 2 * G)