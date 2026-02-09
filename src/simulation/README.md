# simulation

## Common Issues

### Missing terrain.dae file

You may run into issues with a missing terrain.dae file. The issue is that the `terrain.dae` file is stored in **Git LFS** (Large File Storage), but hasn't been downloaded. To fix this, do the following:

1. Install git-lfs if not already installed
```bash
sudo apt install git-lfs
```

2. Initialize git-lfs in your local repo
```bash
git lfs install
```

3. Pull the actual LFS files
```bash
git lfs pull
```

### Ground truth TF publisher not found

If you get an error like `executable 'ground_truth_tf_publisher.py' not found on the libexec directory` when launching with `publish_ground_truth_tf:=true`, the script needs executable permissions.

Fix it by running:
```bash
chmod +x src/simulation/scripts/ground_truth_tf_publisher.py
colcon build --packages-select simulation
source install/setup.bash
```