# Vegetable Image Classifier

This project performs vegetable image classification using TensorFlow, EfficientNetB0 for transfer learning, and OpenCV for data augmentation and preprocessing. A Streamlit app is also provided for real-time image classification.

## Features

- **Data Preprocessing:**  
  - Blurry image removal using OpenCV.
  - Metadata filtering using Exiftool
  - Color calibration using a ColorChecker Passport.
  - Offline data augmentation using OpenCV.
  
- **Model Training:**  
  - Transfer learning with EfficientNetB0.
  - Fine-tuning of the top layers for improved performance(if needed)
  
- **Evaluation and Visualization:**  
  - Accuracy, loss, confusion matrix, and classification reports.
  - Utility functions for single image predictions and misclassification analysis.
  
- **Streamlit App:**  
  - A user-friendly interface to upload images or use the webcam for real-time classification.

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/Vegetable-Image-Classifier.git
   cd Vegetable-Image-Classifier
