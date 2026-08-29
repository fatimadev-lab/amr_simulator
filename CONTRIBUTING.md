# Contributing to AMR Simulator

Thank you for your interest in contributing to the AMR Simulator project! This document provides guidelines and instructions for contributing.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR-USERNAME/amr_simulator.git
   cd amr_simulator
   ```
3. **Create a virtual environment**:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .\.venv\Scripts\activate
   ```
4. **Install dependencies** (including development dependencies):
   ```bash
   pip install -e ".[dev]"
   ```

## Development Workflow

1. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** and follow the coding standards below

3. **Run tests** to ensure nothing is broken:
   ```bash
   pytest tests/
   ```

4. **Commit your changes** with clear messages:
   ```bash
   git commit -m "Add descriptive message about changes"
   ```

5. **Push to your fork** and create a Pull Request

## Coding Standards

- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guidelines
- Write clear docstrings for all classes and functions
- Include type hints where appropriate
- Add unit tests for new functionality
- Keep functions focused and reasonably sized

## Adding Tests

- Place tests in the `tests/` directory
- Name test files as `test_*.py`
- Use descriptive test function names starting with `test_`
- Aim for good test coverage, especially for core algorithms

Example test structure:
```python
def test_feature_description():
    """Detailed description of what is being tested."""
    # Arrange
    setup_code()
    
    # Act
    result = function_under_test()
    
    # Assert
    assert result == expected_value
```

## Reporting Issues

- Use the GitHub Issues tracker
- Include a clear description of the problem
- Provide steps to reproduce for bugs
- Include Python version and OS information

## Areas for Contribution

We welcome improvements in several areas:

- **Better sensor models** - More realistic LiDAR simulation
- **SLAM algorithms** - Particle filters (FastSLAM), pose-graph optimization
- **Scan matching** - ICP (Iterative Closest Point) algorithms
- **ROS integration** - Bridge for real robot experiments
- **Documentation** - Tutorials, examples, API documentation
- **Performance** - Optimization of bottleneck algorithms
- **Testing** - Additional test coverage and edge cases

## Code Review Process

All submissions require review before merging. Reviewers may suggest changes, improvements, or alternative approaches. This is a normal part of the development process.

## License

By contributing, you agree that your contributions will be licensed under the same MIT License as the project.

## Questions?

Feel free to open an issue or discussion on GitHub with any questions about contributing.

Happy coding! 🚀
