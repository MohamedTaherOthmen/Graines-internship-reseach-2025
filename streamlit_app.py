import streamlit as st
import cv2
import numpy as np
import tensorflow as tf
from PIL import Image
from tensorflow.keras.applications.efficientnet import preprocess_input

# Load the trained model (ensure the model file is in the same directory)
model = tf.keras.models.load_model("best_vegetable_model.h5")

# Class names (must match your training labels)
CLASSES = ['Bean', 'Bitter_Gourd', 'Bottle_Gourd', 'Brinjal', 'Broccoli', 
           'Cabbage', 'Capsicum', 'Carrot', 'Cauliflower', 'Cucumber', 
           'Papaya', 'Potato', 'Pumpkin', 'Radish', 'Tomato']

st.title("Vegetable Classifier Demo")
st.write("Upload an image or use the webcam to classify vegetables.")

# Sidebar: Choose input mode
input_mode = st.sidebar.radio("Select Input Mode", ("Upload Image", "Webcam"))

def preprocess_image(image: np.ndarray) -> np.ndarray:
    # Resize image to target size
    image = cv2.resize(image, (224, 224))
    image = image.astype("float32")
    # Use EfficientNet's preprocessing to scale input to [-1, 1]
    image = preprocess_input(image)
    return image

if input_mode == "Upload Image":
    uploaded_file = st.file_uploader("Choose an image...", type=["jpg", "jpeg", "png"])
    if uploaded_file is not None:
        # Load the image and display it
        image = np.array(Image.open(uploaded_file).convert("RGB"))
        st.image(image, caption="Uploaded Image", use_column_width=True)
        
        # Preprocess the image using our defined function
        input_img = preprocess_image(image)
        input_img = np.expand_dims(input_img, axis=0)
        
        # Get prediction
        preds = model.predict(input_img)
        pred_class = CLASSES[np.argmax(preds)]
        confidence = np.max(preds)
        
        st.write("Prediction:", pred_class)
        st.write("Confidence:", confidence)
elif input_mode == "Webcam":
    st.write("Click 'Start' to use your webcam.")
    run = st.checkbox("Start")
    FRAME_WINDOW = st.image([])
    cap = cv2.VideoCapture(0)
    if run:
        while True:
            ret, frame = cap.read()
            if not ret:
                st.write("Failed to capture image")
                break
            # Convert captured frame from BGR to RGB and display it
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            FRAME_WINDOW.image(frame_rgb)
            
            # Preprocess the frame
            input_frame = preprocess_image(frame_rgb)
            input_frame = np.expand_dims(input_frame, axis=0)
            
            preds = model.predict(input_frame)
            pred_class = CLASSES[np.argmax(preds)]
            confidence = np.max(preds)
            
            # Draw prediction text on the frame and display it in Streamlit
            cv2.putText(frame, f"{pred_class}: {confidence:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            st.write("Prediction:", pred_class, "Confidence:", confidence)
            break
    cap.release()

st.write("✅ Demo complete!")
st.write("📦 Source code saved to 'streamlit_app.py'")
# streamlit run streamlit_app.py (run the instruction in the terminal)
