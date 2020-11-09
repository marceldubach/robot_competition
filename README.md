# bottlebot
Code repository for the bottle bot project


## Create a virtual environment called 'venv':
on the py: run 'virtualenv venv'

## If additional libraries have been installed in venv:
Generate a new requirements as follows:
1) activate the virtual environment as follows:
   navigate to bottlebot directory and run

   .\venv\Scripts\activate (for windows)
   source venv/bin/activate (for linux)
   
2) create the new requirements.txt file running:
   pip freeze > requirements.txt
   
3) run 'deactivate' to quit the virtual environment


## installing requirements:
install the requirements on another machine (venv should be activated!) running:
pip install -r requirements.txt
