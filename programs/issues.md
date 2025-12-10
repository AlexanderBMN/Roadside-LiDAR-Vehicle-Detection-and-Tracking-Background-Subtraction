# Code Modification Summary

## 1. Original Dataloader Limitation

The original dataloader only supported Velodyne **PCAP** files.  
**Fix:** Updated pipeline to support generic **PCD** files instead.

## 2. Restricted Training/Testing Subset

The previous implementation processed only a preselected subset of frames.  
**Fix:** Modified loops to iterate over the **entire dataset**.

## 3. Visualization Disabled

GUI-based visualization (`pcplayer`, etc.) was removed.  
**Reason:** No functional impact, improves stability and speed in headless execution.

## 4. Incorrect or Legacy Variable Names

Several variables were incorrectly named or inconsistent.  
**Fixes:**

- `total_channel`, `total_grid` → replaced with `vert_total_grids`, `horiz_total_grids`
- `azimuth`, `elevation` → replaced with `azimuths`, `elevations` for clarity and correctness

## 5. Invalid Zero Index Usage in MATLAB

MATLAB does not allow zero-based indexing.  
**Fix:** Added safety checks:

```matlab
if grid_idx == 0 || channel_idx == 0
    back_idxes(pnt_idx) = 1;
    continue;
end
```

Prevents invalid matrix access and mirrors training logic.

## 6. Removal of Duplicated Logic

Loading point clouds and converting to spherical coordinates in testing was duplicated and removed.

## 7. Missing `dbcanDenoise` Function

The function `dbcanDenoise` does not exist in MATLAB. Even if it was a spelling mistake, no equivalent function is
available.  
**Fix:** Disabled this call.

## 9. Irregular Scan Patterns Not Supported

If the LiDAR sensor does not produce uniformly spaced vertical scan lines,  
the hash-based angular indexing approach becomes invalid.  
Current method assumes **uniform LiDAR scanning**.

---

## Additional Adjustments

Custom parameters were tuned to match our Dataset characteristics:

- Z-axis cropping range
- `FOV_vert`
- `vert_total_grids` and `horiz_total_grids`
- `azimuth_resltn`
- Dataset root paths  
