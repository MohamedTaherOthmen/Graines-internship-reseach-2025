# Final Study Projects | Graines d'entrepreneurs  
**Robotics & AI Solutions for Sustainable Innovation**  

This repository contains **a research project** developed as part of the **Final Study Projects** at **Graines d'entrepreneurs**. This project includes **Embedded Systems (ROS)** and **AI/ML** components.  

---

## 🌟 Projects Overview  

| Project          | Description                                                                                   |  
|-------------------|-----------------------------------------------------------------------------------------------|  
| **SMdrone**       | Agricultural drone for olive grove monitoring, disease detection, and yield estimation using advanced imaging. |  
| **SDrone**        |  urveillance drone with facial recognition to enforce anti-littering laws in public spaces via automated fines. |  
| **CleaningOcean** | Autonomous boat collecting ocean plastic waste and environmental data with obstacle detection |  
| **SaveEat**       | Robot scanning restaurant tables to penalize food waste via app-linked surcharges.|  

---

## 📂 Repository Structure  
```bash  
├── SMdrone/  
│   ├── EmbeddedSystems/          
│   └── IA/                        
├── SDrone/  
│   ├── EmbeddedSystems/          
│   └── IA/                      
├── CleaningOcean/  
│   ├── EmbeddedSystems/         
│   └── IA/            
├── SaveEat/  
│   ├── EmbeddedSystems/        
│   └── IA/            
├──        
```  

---

## 🛠️ Technologies Used  

### **Embedded Systems (ROS)**  
- **ROS 1 Noetic**: Navigation, sensor integration, Gazebo simulation, RVIZ Simulation  
- **Tools**: SolidWorks (3D design)  

### **AI/ML**  
- **Frameworks**: TensorFlow, PyTorch, OpenCV  
- **Tasks**: A high-accuracy computer vision solution  for automated classification of fruits and vegetables.  
- **Accuracy**: Models achieved **95–99%** accuracy across projects.  

---

## ⚙️ Setup & Installation  

### Prerequisites  
- **ROS 1 Noetic** (Ubuntu 20.04).  
- **Python 3.8+** with `virtualenv`.  
- **TensorFlow/PyTorch** for AI components.  

### Steps  
1. Clone the repository:  
   ```bash  
   git clone hhttps://github.com/MohamedTaherOthmen/Graines-internship-reseach-2025.git  
   cd graines-projects  
   ```  

2. **Embedded Systems (ROS)**:  
   ```bash  
   cd SMdrone/Embedded_Systems  
   catkin_make                  # Build ROS workspace  
   source devel/setup.bash  
   roslaunch <package_name> gazebo.launch  # Launch Gazebo simulation  
   cd scripts
   python3 telop_robot.py # Run python code
   ```  

3. **AI/ML Models**:  
   ```bash  
   cd SMdrone/IA  
   jupyter <notebook_name>  
   ```  

---

## 👥 Contributors  
- **Robotics Program Manager**: Mohamed Taher Othmen  
- **Interns** 
- **Graines d'entrepreneurs**: https://www.facebook.com/grainesdentrepreneurstunisie/?locale=fr_FR  


**🚀 Note**: For hardware schematics or proprietary datasets, contact [Mohamed Taher Othmen](mailto:mohamedtaher.othmant@email.com).  

--- 
