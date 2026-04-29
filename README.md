# sEMG Adaptive Rehabilitation Hand
An sEMG-based adaptive rehabilitation hand that combines deep learning gesture classification with intensity prediction to drive impedance-based, power-adaptive actuation.

## Highlights
- Deep learning classification of hand gestures plus continuous intensity prediction (0.0 to 1.0).
- Adaptive impedance control that modulates target motion based on detected contraction and predicted intensity.
- Sliding-window filtering (majority vote + median) to stabilize predictions.
- Exportable Arduino-ready model header (`dual_model.h`) for embedded inference.
- Test data generation and replay using fixed sEMG windows for validation.

## System Overview
1. **Data pipeline**: Extract 15 handcrafted features (MAV, WL, RMS, ZC, SSC across 3 channels) from 375-sample windows.
2. **Training**: Train a classifier for gesture labels and a regressor for contraction intensity.
3. **Export**: Emit a compact C++ header (`dual_model.h`) and scaler stats for embedded use.
4. **Firmware**: Run inference on-window, filter predictions, then drive impedance control to adjust cable displacement.

## Repository Layout
- [EMG_Modified/test_emg_data.h](EMG_Modified/test_emg_data.h): Test windows and labels used by embedded code.
- [EMG_Modified/dual_model.h](EMG_Modified/dual_model.h): Exported classifier + regressor weights and inference functions.
- [src/FlexoGrip.ino](src/FlexoGrip.ino): ESP32-S3 firmware with inference, filtering, and impedance control.
- [notebook/Rehab_Final_Project_Team_4_EMG_Hand_Movement_Classifcation.ipynb](notebook/Rehab_Final_Project_Team_4_EMG_Hand_Movement_Classifcation.ipynb): Training and export workflow.

## Quick Start (Recommended Flow)
1. Open the notebook and run the training + export sections to generate `dual_model.h` and scaler stats.
2. Generate test windows (or update existing test data) and place them in `EMG_Modified/test_emg_data.h`.
3. Ensure `dual_model.h` and `test_emg_data.h` are in the include path for the firmware.
4. Build and upload [src/FlexoGrip.ino](src/FlexoGrip.ino) to the ESP32-S3.

## Firmware Notes
- **Gesture decision**: 3-window majority vote to reduce misclassification.
- **Intensity**: 3-window median for stable intensity control.
- **Adaptive power control**: Target displacement is computed from predicted intensity and used by the impedance controller.

## Data Alignment Note
If you reorder `test_labels`, the corresponding entries in `test_emg_data` must be reordered to keep the labels aligned with their windows.
