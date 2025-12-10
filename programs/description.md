# Overview of the Hash-Based Background Subtraction Pipeline

This document summarizes the two major phases of the MATLAB hash‑based LiDAR background subtraction system: **Training**
and **Inference**.


---

# ✅ TRAINING PHASE — Learning the Background Threshold Matrix

## 1. Load Training PCD Files

Sequentially read all `.pcd` frames used for building the static background model.

---

## 2. ROI Cropping (Outlier Removal)

Apply a fixed 3D region-of-interest filter:

```
xlimits, ylimits, zlimits
```

Removes points outside the usable scene before further processing.

---

## 3. Convert Each Point to Spherical Coordinates

### 3.1 Point‑wise conversion using MATLAB:

```matlab
[azimuth, elevation, range] = cart2sph(x, y, z);
```

### 3.2 Normalize azimuth to `[0, 360]°`

```matlab
degree = rad2deg(azimuth);
degree(degree < 0) = degree(degree < 0) + 360;
```

---

## 4. Map Each Point Into a 2D Angular Grid

The LiDAR frame is discretized into a **(vertical × horizontal)** grid.

### Horizontal bin index:

```matlab
grid_idx = floor(azimuth_degree / azimuth_unit) + 1;
```

### Vertical bin index:

```matlab
channel_idx = floor(elevation_degree / elevation_resltn) + 1;
```

---

## 5. Fill the 3D Range Tensor

For each grid cell and each frame:

```
range_matrix(channel_idx, grid_idx, frame) = min(range)
```

Stores the *closest* point observed over time at that angular location.

---

## 6. Learn a Threshold per Cell

For each `(channel, grid)` cell, extract all temporal distances:

```
distances = range_matrix(channel_idx, grid_idx, :)
```

Then compute a robust background threshold:

```
thr = thresholding(distances)
range_thrld_matrix(channel_idx, grid_idx) = thr
```

This produces a full **static background model**.

---

---

# 🟦 INFERENCE PHASE — Generating Foreground Masks

## 1. Load Testing PCD Frame

## 2. Apply ROI Cropping

## 3. Compute Spherical Coordinates

## 4. Compute Horizontal Bin (azimuth)

## 5. Compute Vertical Bin (elevation)

## 6. Background / Foreground Classification

## 7. Extract Foreground Points

## 8. Statistical Outlier Removal - pcdenoise

`pcdenoise(ptCloud_obj)` performs a statistical outlier removal procedure:

- For each point, a local neighborhood is estimated (using either radius-based search or k-nearest neighbors).
- The algorithm computes the mean distance between a point and its neighbors.
- Points whose mean distance significantly exceeds the statistical distribution are classified as **outliers** and
  removed.

## 9. Save Foreground Point Clouds
