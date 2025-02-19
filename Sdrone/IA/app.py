import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import load_model
import streamlit as st
import numpy as np

# Load the trained model and compile it
model = load_model('C:\\Users\\MSI\\Desktop\\test\\final_vegetable_model.keras')
model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])  # Ensure model is compiled

# Define category labels and emojis
data_cat = [
    ['apple', '🍎'], ['banana', '🍌'], ['beetroot', '🌱'], ['bell pepper', ' '],
    ['cabbage', '🥬'], ['capsicum', ' '], ['carrot', '🥕'], ['cauliflower', '🌸'],
    ['chilli pepper', '🌶️'], ['corn', '🌽'], ['cucumber', '🥒'], ['eggplant', '🍆'],
    ['garlic', '🧄'], ['ginger', ' '], ['grapes', '🍇'], ['jalepeno', '🌶️'],
    ['kiwi', '🥝'], ['lemon', '🍋'], ['lettuce', '🥬'], ['mango', '🥭'],
    ['onion', '🧅'], ['orange', '🍊'], ['paprika', '🌶️'], ['pear', '🍐'],
    ['peas', '🟢'], ['pineapple', '🍍'], ['pomegranate', ' '], ['potato', '🥔'],
    ['raddish', '🌱'], ['soy beans', ' '], ['spinach', '🥬'], ['sweetcorn', '🌽'],
    ['sweetpotato', '🍠'], ['tomato', '🍅'], ['turnip', '🌱'], ['watermelon', '🍉']
]

img_height = 224
img_width = 224

# Streamlit UI Setup
st.set_page_config(page_title="Fruit & Vegetable Classifier", layout="centered")

st.sidebar.title("About the Model")
st.sidebar.markdown("""
    - **Type**: Image Classification  
    - **Model**: Convolutional Neural Network (CNN)  
    - **Framework**: TensorFlow/Keras  
    - **Classes**: 36 Categories  
""")

st.title("🌟 Fruit & Vegetable Classifier 🌟")
st.write("Upload an image of a fruit or vegetable, and this app will classify it for you!")

# File uploader
uploaded_file = st.file_uploader("Choose an image", type=["jpg", "jpeg", "png"])

if uploaded_file:
    try:
        # Load and preprocess the image
        image_load = tf.keras.utils.load_img(uploaded_file, target_size=(img_height, img_width))
        img_arr = tf.keras.utils.img_to_array(image_load)  # Convert image to array
        img_arr = img_arr / 255.0  # Normalize pixel values
        img_bat = np.expand_dims(img_arr, axis=0)  # Add batch dimension

        # Model prediction
        predict = model.predict(img_bat)
        predicted_index = np.argmax(predict)  # Get class index
        predicted_class, emoji = data_cat[predicted_index]  # Get class name and emoji
        confidence = np.max(predict) * 100  # Get confidence score

        # Display results
        st.image(uploaded_file, width=250, caption="Uploaded Image")
        st.markdown(f"### Predicted Category: **{predicted_class}** {emoji}")
        st.progress(int(confidence))
        st.markdown(f"### 🎯 Confidence: **{confidence:.2f}%**")

    except Exception as e:
        st.error(f"An error occurred: {str(e)}")
else:
    st.info("👆 Upload an image file to get started!")
