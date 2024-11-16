# Basic Motion Controller
from controller import Robot, Motor, Camera
import numpy as np

class JetBotController:
    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        
        # Set initial motor parameters
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Initialize camera
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        
    def set_motors(self, left_speed, right_speed):
        """Set motor speeds"""
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
    def get_camera_image(self):
        """Get image from camera"""
        image = self.camera.getImage()
        return np.frombuffer(image, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))

    def basic_motion_test(self):
        """Test basic motions"""
        # Move forward
        self.set_motors(5.0, 5.0)
        self.robot.step(2000)  # Run for 2 seconds
        
        # Turn right
        self.set_motors(5.0, -5.0)
        self.robot.step(1000)  # Run for 1 second
        
        # Move forward again
        self.set_motors(5.0, 5.0)
        self.robot.step(2000)
        
        # Stop
        self.set_motors(0.0, 0.0)

# Data Collection for Collision Avoidance
class DataCollector(JetBotController):
    def __init__(self):
        super().__init__()
        self.data_dir = "collision_avoidance_data"
        self.categories = ["free", "blocked"]
        
    def collect_data(self, num_samples=20):
        """Collect training data for collision avoidance"""
        import os
        import cv2
        
        # Create directories if they don't exist
        for category in self.categories:
            os.makedirs(os.path.join(self.data_dir, category), exist_ok=True)
        
        for category in self.categories:
            count = 0
            while count < num_samples:
                # Get image from camera
                image = self.get_camera_image()
                
                # Save image
                filename = f"{category}_{count}.jpg"
                filepath = os.path.join(self.data_dir, category, filename)
                cv2.imwrite(filepath, cv2.cvtColor(image, cv2.COLOR_RGBA2BGR))
                
                count += 1
                self.robot.step(self.timestep)

# Collision Avoidance using AI
class CollisionAvoidance(JetBotController):
    def __init__(self):
        super().__init__()
        self.model = self.load_model()
        
    def load_model(self):
        """Load the trained collision avoidance model"""
        import torch
        import torchvision
        
        model = torchvision.models.resnet18(pretrained=False)
        model.fc = torch.nn.Linear(512, 2)  # Binary classification: free or blocked
        model.load_state_dict(torch.load('collision_avoidance_model.pth'))
        model.eval()
        return model
    
    def process_image(self, image):
        """Process image for model input"""
        import torch
        import torchvision.transforms as transforms
        
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        
        # Convert to RGB and apply transforms
        image_rgb = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        return transform(image_rgb).unsqueeze(0)
    
    def run_collision_avoidance(self):
        """Main collision avoidance loop"""
        while self.robot.step(self.timestep) != -1:
            # Get camera image
            image = self.get_camera_image()
            
            # Process image and get prediction
            with torch.no_grad():
                x = self.process_image(image)
                y = self.model(x)
                
            # Get prediction (0: blocked, 1: free)
            prediction = torch.argmax(y).item()
            
            # Control robot based on prediction
            if prediction == 1:  # Path is free
                self.set_motors(5.0, 5.0)  # Move forward
            else:  # Path is blocked
                self.set_motors(5.0, -5.0)  # Turn right
                self.robot.step(500)  # Turn for 0.5 seconds

def main():
    # Test basic motion
    print("Testing basic motion...")
    controller = JetBotController()
    controller.basic_motion_test()
    
    # Collect training data
    print("Collecting training data...")
    collector = DataCollector()
    collector.collect_data(num_samples=20)
    
    # Run collision avoidance
    print("Running collision avoidance...")
    avoider = CollisionAvoidance()
    avoider.run_collision_avoidance()

if __name__ == "__main__":
    main()
