import numpy as np
from scipy.spatial import ConvexHull
from scipy.integrate import quad


X_LOW = -0.5
X_HIGH = 0.5
Y_LOW = -0.5
Y_HIGH = 0.5
Z_LOW = -0.5
Z_HIGH = 0.5

# # Define the pdf
# def probability_density_function(x, y, z):
#     if x >= 0 and x <= 0.5:
#         return np.zeros(1)
#     return np.array([np.exp(-(x**2 + y**2 + z**2))])


# Set the pdf to 0 if there is an obstacle (NB: hardcoded)
def picewise_to_zero(x, y, z):
    if X_LOW < x < X_HIGH and Y_LOW < y < Y_HIGH and Z_LOW < z < Z_HIGH:
        return True
    return False


# Multivariate Gaussian PDF in R^3
def probability_density_function(x, y, z):
    # if picewise_to_zero:
    #     return np.zeros(1)

    mean = np.zeros(3)  # Mean vector
    covariance = np.eye(3)  # Covariance matrix

    point = np.array([x, y, z])
    exponent = -0.5 * np.dot(np.dot((point - mean).T, np.linalg.inv(covariance)), (point - mean))
    normalization = (2 * np.pi) ** (-len(mean) / 2) * np.linalg.det(covariance) ** (-0.5)

    return np.array([normalization * np.exp(exponent)])


# Centroid pdf
def centroid_pdf(x, y, z):
    return np.array([x,y,z]) * probability_density_function(x,y,z)


# Compute the volume of the polyhedron using convex hull.
def polyhedron_volume(vertices):
    hull = ConvexHull(vertices)
    return hull.volume


# Find Voronoi cell mass
def integrate_density_over_polyhedron(pdf_func, vertices):
    # Compute the volume of the polyhedron
    polyhedron_vol = polyhedron_volume(vertices)

    # Define the function to integrate, which is the product of the density function and the volume element
    def integrand(x, y, z):
        return pdf_func(x, y, z)

    # Define the integration limits
    limits = [(min(coord), max(coord)) for coord in zip(*vertices)]

    # Perform the triple integral over the polyhedron
    result = [quad(lambda z: quad(lambda y: quad(lambda x: integrand(x, y, z)[i], 
                     limits[0][0], 
                     limits[0][1])[0], 
                     limits[1][0], 
                     limits[1][1])[0], 
                     limits[2][0], 
                     limits[2][1])[0]
                    for i in range(len(integrand(0,0,0)))
              ]
    # Multiply the result by the volume of the polyhedron
    return np.array(result) * polyhedron_vol


def main():
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    # vertices = np.array([[1, 1, 1], [3, 0, 0], [0, 3, 0], [3, 3, 3]])
    # vertices = np.array([[0,0,0], [1,0,0], [1,0,1], [0,0,1], [0,1,0], [1,1,0], [1,1,1], [0,1,1]])

    mv = integrate_density_over_polyhedron(probability_density_function, vertices)
    print("M_v =", mv)

    cv = integrate_density_over_polyhedron(centroid_pdf, vertices) 
    print("C_v =", cv)


if __name__ == "__main__":
    main()
