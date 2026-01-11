# Athena Code

This repository contains all the code for UMDLoop's 2025-26 rover, Athena.

## How To Contribute

### Learning Git

If you're new to Git and GitHub, start with this beginner-friendly tutorial: [GitHub's Hello World Guide](https://docs.github.com/en/get-started/quickstart/hello-world)

For a more comprehensive introduction, check out: [Git and GitHub Tutorial for Beginners](https://www.freecodecamp.org/news/git-and-github-for-beginners/)

### Setting up Git

Before you can contribute, you'll need to have Git installed and configured on your machine:

1. **Install Git** - Follow GitHub's guide to [set up Git](https://docs.github.com/en/get-started/git-basics/set-up-git) on your system
2. **Configure Git** - Set your username and email that will be associated with your commits

### SSH Keys

To securely authenticate with GitHub without entering your password each time, set up SSH keys:

1. **Check for existing SSH keys** - See if you already have SSH keys: [Checking for existing SSH keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)
2. **Generate a new SSH key** - If needed, create a new SSH key pair: [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
3. **Add SSH key to GitHub** - Upload your public key to your GitHub account: [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### Making a Pull Request (PR)

Once you're set up, here's how to contribute your changes:

1. **Fork the repository** - Create your own copy of the repository: [Fork a repo](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo)

2. **Clone your fork** - Download your forked repository to your local machine using `git clone`.

3. **Create a feature branch** - Make a new branch for your specific feature or fix: [Creating and deleting branches](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository)
```bash
   git checkout -b feature/your-feature-name
```

4. **Make your changes** - Write your code, test it thoroughly, and commit your changes

5. **Push to your fork** - Upload your feature branch to your GitHub fork
```bash
   git push origin feature/your-feature-name
```

6. **Open a Pull Request** - Submit your changes for review: [Creating a pull request from a fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork)

Your PR will be reviewed by your lead, and once approved, it will be merged into the main codebase!

### Workspace Startup:

1. **Navigate to your workspace**

2. **Install required dependencies**
```bash
rosdep install --from-paths src y --ignore-src
```

3. **Build the workspace**
```bash
colcon build --symlink-install
```

4. **Set up can0**

_Hardware:_
```bash
./src/athena-code/src/tools/scripts/can_setup.sh 
```

_Virtual:_
```bash
./src/athena-code/src/tools/scripts/virtual_can_setup.sh
```

5. **Launch a subsystem!**
```bash
./src/athena-code/src/tools/scripts/<subsystem>_launch.sh
```
