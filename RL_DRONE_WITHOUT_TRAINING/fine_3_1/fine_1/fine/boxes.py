import os
import random
import time

def generate_random_rgba():
    r = random.random()
    g = random.random()
    b = random.random()
    a = 1.0  # Full opacity
    return f"{r:.2f} {g:.2f} {b:.2f} {a:.2f}"

def generate_box_model(name, x, y, z, color):
    return f"""
    <model name="{name}">
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <gravity>0</gravity>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """

def add_boxes_to_world(template_path, new_sdf_path, num_boxes=500):
    # Step 1: Delete existing baylands.sdf if it exists
    if os.path.exists(new_sdf_path):
        os.remove(new_sdf_path)
        print("🗑️ Deleted old baylands.sdf")

    # Step 2: Read clean base template
    with open(template_path, 'r') as f:
        base_sdf = f.read()

    # Step 3: Generate box models with random colors
    x_bounds = (-250,250)
    y_bounds = (-250,250)
    z_bounds = (10, 12)

    timestamp = int(time.time())
    models = ""
    for i in range(num_boxes):
        x = random.uniform(*x_bounds)
        y = random.uniform(*y_bounds)
        z = random.uniform(*z_bounds)
        model_name = f"box_{i}_{timestamp}"
        color = generate_random_rgba()
        models += generate_box_model(model_name, x, y, z, color)

    # Step 4: Inject models before </world>
    if "</world>" not in base_sdf:
        print("❌ </world> tag not found in the template!")
        return

    final_sdf = base_sdf.replace("</world>", models + "\n</world>")

    # Step 5: Write to new baylands.sdf
    with open(new_sdf_path, 'w') as f:
        f.write(final_sdf)

    print(f"✅ Created new baylands.sdf with {num_boxes} colored boxes at: {new_sdf_path}")


# Paths
template_path = "/home/c3ilab/PX4-Autopilot/Tools/simulation/gz/worlds/default (copy).sdf"
new_sdf_path = "/home/c3ilab/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"
# template_path="/home/parth/PX4-Autopilot/Tools/simulation/gz/worlds/default (copy).sdf"
# new_sdf_path="/home/parth/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"
add_boxes_to_world(template_path, new_sdf_path, num_boxes=400)
