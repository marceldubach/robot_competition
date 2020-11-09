# bottlebot
Code repository for the bottle bot project


## Create a virtual environment called 'venv':
On the raspberry pi run: `$ virtualenv venv`

This creates a virtual environment. (The environment may be created inside the git
  repository, the directory venv is included in the .gitignore and will not be synced)

All required libraries can be installed in the virtual environment (once it is activated!) running
`pip install -r requirements.txt`

### How to activate the virtual environment
On Windows, run `\venv\Scripts\activate`
On Linux, run `source venv/bin/activate`

In order to deactivate the virtual environment, just run `deactivate`

If additional libraries have been installed, create a the new requirements running
`pip freeze > requirements.txt`
