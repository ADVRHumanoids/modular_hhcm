# Web Server Gateway Interface (WSGI)
from src.modular.web.RobotDesignStudio import app

if __name__ == "__main__":
    app.run(threaded=True)
