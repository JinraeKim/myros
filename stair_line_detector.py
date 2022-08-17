from copy import copy
import cv2
import glob
from pathlib import Path

import numpy as np
from numpy.random import default_rng
rng = default_rng()


class RANSAC:
    """
    Borrowed from https://en.wikipedia.org/wiki/Random_sample_consensus#Example_Code
    """
    # def __init__(self, n=10, k=100, t=0.05, d=10, model=None, loss=None, metric=None):
    def __init__(self, n=10, k=100, t=0.005, d=10, model=None, loss=None, metric=None):
        self.n = n              # `n`: Minimum number of data points to estimate parameters
        self.k = k              # `k`: Maximum iterations allowed
        self.t = t              # `t`: Threshold value to determine if points are fit well
        self.d = d              # `d`: Number of close data points required to assert model fits well
        self.model = model      # `model`: class implementing `fit` and `predict`
        self.loss = loss        # `loss`: function of `y_true` and `y_pred` that returns a vector
        self.metric = metric    # `metric`: function of `y_true` and `y_pred` and returns a float
        self.best_fit = None
        self.best_error = np.inf
        self.inlier_indices = None

    def fit(self, X, y):
        for _ in range(self.k):
            ids = rng.permutation(X.shape[0])

            maybe_inliers = ids[: self.n]
            maybe_model = copy(self.model).fit(X[maybe_inliers], y[maybe_inliers])

            thresholded = (
                self.loss(y[ids][self.n :], maybe_model.predict(X[ids][self.n :]))
                < self.t
            )

            inlier_ids = ids[self.n :][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_indices = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(X[inlier_indices], y[inlier_indices])

                this_error = self.metric(
                    y[inlier_indices], better_model.predict(X[inlier_indices])
                )

                if this_error < self.best_error:
                    self.best_error = this_error
                    self.best_fit = maybe_model
                    self.inlier_indices = inlier_indices
        #     if len(inlier_ids) != 0:
        #         print(f"inlier_ids: {inlier_ids}")
        #         import pdb; pdb.set_trace()
        #
        # if self.best_fit is None:
        #     import pdb; pdb.set_trace()

        return self

    def predict(self, X):
        return self.best_fit.predict(X)


def square_error_loss(y_true, y_pred):
    # NOTE: speicialised to deal with angles
    return wraptopi(y_true - y_pred) ** 2


def mean_square_error(y_true, y_pred):
    return np.sum(square_error_loss(y_true, y_pred)) / y_true.shape[0]


def wraptopi(angles):
    """
    Convert angles into [-pi, pi)
    """
    return (angles + np.pi) % (2*np.pi) - np.pi


class LinearRegressor:
    def __init__(self):
        self.params = None

    def fit(self, X: np.ndarray, y: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        self.params = np.linalg.inv(X.T @ X) @ X.T @ y
        return self

    def predict(self, X: np.ndarray):
        r, _ = X.shape
        X = np.hstack([np.ones((r, 1)), X])
        return X @ self.params


class StairLineDetector:
    def __init__(self):
        """
        `regressor` is a RANSAC regressor.
        """
        # self.th1 = 50
        # self.th2 = 200
        self.th1 = 50
        self.th2 = 100

    def file2img(self, file: str):
        """
        file: image file location (str)
        """

    def find_edges_and_lines(self, img):
        """
        Input:
            img: cv2 image, e.g., `img = cv2.imread(file)`
        Output:
            lines: not line segmentations, just lines
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(
            gray, self.th1, self.th2,
            apertureSize = 3,
        )
        # lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 150)
        # lines = cv2.HoughLinesP(
        #     edges, 1, np.pi/180, 80, 30, 1,
        # )  # NOTE: HoughLinesP didn't work well
        return edges, lines

    def visualise(self, img, save_fig=True):
        """
        CAUTION: it will override `img`.
        """
        edges, lines = self.find_edges_and_lines(img)
        for line in lines:
            for r, theta in line:  # for HoughLines
                a, b = np.cos(theta), np.sin(theta)
                x0 = r * a
                y0 = r * b
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            # for x1, x2, y1, y2 in line:  # for HoughLinesP
            #     cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)
        if save_fig:
            # save_file = "line_" + file
            # Path(save_file).parents[0].mkdir(exist_ok=True, parents=True)
            cv2.imwrite("lines.png", img)
            cv2.imwrite("edges.png", edges)



if __name__ == "__main__":
    detector = StairLineDetector()
    # file = "record_t3_2022-08-17-14-26-48/color/frame000100.png"
    # file = "frame000038.png"
    file = "frame000200.png"
    img = cv2.imread(file)
    _, lines = detector.find_edges_and_lines(img)
    detector.visualise(img)

    rs = np.array([line[0][0] for line in lines]).reshape(-1, 1)
    thetas = np.array([line[0][1] for line in lines]).reshape(-1, 1)
    # regression
    n = int(rs.shape[0] / 2)  # parameter for the number of inliers
    d = int(rs.shape[0] / 2)  # parameter for the number of inliers
    regressor = RANSAC(model=LinearRegressor(), loss=square_error_loss, metric=mean_square_error)
    X, y = rs, thetas
    regressor.fit(X, y)

    import matplotlib.pyplot as plt
    plt.style.use("seaborn-darkgrid")
    fig, ax = plt.subplots(1, 1)
    ax.set_box_aspect(1)

    plt.scatter(X, y)
    plt.scatter(X[regressor.inlier_indices], y[regressor.inlier_indices])

    line = np.linspace(np.min(X), np.max(X), num=100).reshape(-1, 1)
    print(f"The best params are: {regressor.best_fit.params}")
    thetas_readable = wraptopi(-(thetas + 0.5*np.pi))  # human readable; x, y for left to right and down to up, resp.
    print(f"The predicted stair angle is: {np.mean(np.rad2deg(thetas_readable[regressor.inlier_indices]))} [deg]")
    plt.plot(line, regressor.predict(line), c="peru")
    plt.xlabel("r")
    plt.ylabel("theta (related to line slopes)")
    plt.savefig("inlier_outlier.png")
