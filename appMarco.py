from fastapi import Body, FastAPI, Request, Form # importare FastAPI ci consente di poter utilizzare tutte le dipendenze che abbiamo installato nel nostro ambiente di FastAPI.
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from starlette.staticfiles import StaticFiles
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
import time
from starlette import status

isFollowing = False

def callback_isFollowing(data):
    global isFollowing
    isFollowing = data.data

rospy.init_node('userCommands', anonymous=True)
pub_start = rospy.Publisher('user/start', Bool, queue_size=1)
pub_stop = rospy.Publisher('user/stop', Bool, queue_size=1)
pub_resume = rospy.Publisher('user/resume', Bool, queue_size=1)
rospy.Subscriber('/isFollowing', Bool, callback_isFollowing)

pub_postpone = rospy.Publisher('user/postpone', String, queue_size=1)
pub_reset_postpone = rospy.Publisher('user/reset_postpone', Bool, queue_size=1)
pub_back_to_home = rospy.Publisher('user/back_to_home', Bool, queue_size=1)

pub_save_path = rospy.Publisher('user/save_path', Bool, queue_size=1)
pub_repeat_saved_path = rospy.Publisher('user/repeat_saved_path', Bool, queue_size=1)



app = FastAPI()
templates = Jinja2Templates(directory="templates")
app.mount("/static", StaticFiles(directory="static"), name="static") # Questa riga mi serve per importare poi tutti i file javascript nel file map.html



@app.get("/map/", response_class=HTMLResponse)
async def map(request: Request):
    context = {'request': request}
    return templates.TemplateResponse("map3.html", context)

@app.post("/start/")
async def startFollowing(request: Request):
    if isFollowing == False:
        pub_start.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("start_button.html", context)

@app.post("/postpone/")
async def postponeFollowing(request: Request, ora = Form(...), minuto = Form(...)):
    if isFollowing == False:
        orario = ora +":" +minuto
        pub_postpone.publish(orario)
    context = {'request': request}
    return templates.TemplateResponse("postpone_button.html", context)

@app.post("/reset_postpone/")
async def postponeFollowing(request: Request):
    pub_reset_postpone.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("reset_postpone_button.html", context)

@app.post("/stop/")
async def stopFollowing(request: Request):
    pub_stop.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("stop_button.html", context)

@app.post("/resume/")
async def resumeFollowing(request: Request):
    pub_resume.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("resume_button.html", context)

@app.post("/back_to_home/")
async def resumeFollowing(request: Request):
    pub_back_to_home.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("back_to_home_button.html", context)

@app.post("/save_path/")
async def resumeFollowing(request: Request):
    pub_save_path.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("save_path_button.html", context)

@app.post("/repeat_saved_path/")
async def resumeFollowing(request: Request):
    pub_repeat_saved_path.publish(True)
    context = {'request': request}
    return templates.TemplateResponse("repeat_saved_path_button.html", context)

@app.post("/do_not_repeat_saved_path/")
async def resumeFollowing(request: Request):
    pub_repeat_saved_path.publish(False)
    context = {'request': request}
    return templates.TemplateResponse("do_not_repeat_saved_path_button.html", context)