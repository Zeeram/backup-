#!/usr/bin/env python3
import rospy
import rospkg
import sys
from std_msgs.msg import Empty, String, Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import subprocess
import time
import json
import pickle
import threading
from geometry_msgs.msg import PoseStamped
import wave
import contextlib
from mutagen.mp3 import MP3

# adding path for python3 files
try:
    sys.path.insert(0, "/home/asimov/catkin_build_ws/install/lib/python3/dist-packages")
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    print("SOME PATH PROBLEM")

from google_speech import Speech


class flow():
    def __init__(self):

        # varaiables to store dynamic process status
        self.ui_process_started = True

        self.face_recognized = ""
        self.face_name = ""
        self.mask_status = False
        self.registered = False
        self.guidance_tour = True
        self.guidance_count = 0
        self.current_process = 0
        self.process_after_faq = 1

        self.person_speaked = False

        self.animation_pub = rospy.Publisher("/animation_data", String, queue_size=2)
        self.ui_control = rospy.Publisher("/ui_data", String, queue_size=1)
        self.person_control = rospy.Publisher("/person_control", String, queue_size=1)
        self.train_control = rospy.Publisher("/train_control", String, queue_size=1)
        self.face_control = rospy.Publisher("/face_control", String, queue_size=1)
        self.mask_control = rospy.Publisher("/mask_control", String, queue_size=1)
        self.temp_control = rospy.Publisher("/temperature_control", String, queue_size=1)
        self.update = rospy.Publisher("/texts_update", String, queue_size=1)
        self.popup_close_pub = rospy.Publisher("/ui_close", String, queue_size=1)
        self.voice_input = rospy.Publisher("/voice_control", String, queue_size=1)
        self.tray_cnt = rospy.Publisher("/tray_control", String, queue_size=1)
        self.menu_input = rospy.Publisher("/menu_control", String, queue_size=1)
        self.hotword = rospy.Publisher("/hotword_control", String, queue_size=1)
        self.button_control_pub = rospy.Publisher("/button_control", String, queue_size=1)
        self.capture_pub = rospy.Publisher("/capture_control", String, queue_size=1)
        self.faq_pub = rospy.Publisher("/speech/state", String, queue_size=1)
        self.capture_button = rospy.Publisher("/capture_button", String, queue_size=1)
        self.battery = rospy.Publisher("/battery_percentage", String, queue_size=1)

        self.goal_publish = rospy.Publisher("move_base_simple/goal", PoseStamped)
        self.cancel_goal = rospy.Publisher("move_base/cancel", String)

        self.ui_textsupdate = rospy.Publisher("/texts_update", String, queue_size=1)

        self.check_entry = rospy.Publisher("/checkentry_control", String, queue_size=1)
        self.long_speech_gesture = rospy.Publisher('/long_speech_gesture', String, queue_size=10)
        rospy.Subscriber('/checkentry_control_result', String, self.checkentry_cb, queue_size=1)
        rospy.Subscriber('/way_cmp', String, self.nav_cb, queue_size=1)

        rospy.Subscriber('/voice_control_result', String, self.voice_cb, queue_size=1)
        rospy.Subscriber('/train_result', String, self.train_cb, queue_size=1)
        rospy.Subscriber('/mic', String, self.mic_cb, queue_size=1)
        rospy.Subscriber('/menu_control_result', String, self.menu_cb, queue_size=1)
        rospy.Subscriber('/hotword_control_result', String, self.hotword_cb, queue_size=1)
        rospy.Subscriber('/button_control_result', String, self.button_control_cb, queue_size=1)
        rospy.Subscriber('/person_result', String, self.person_cb, queue_size=1)
        rospy.Subscriber('/face_result', String, self.face_cb, queue_size=1)
        rospy.Subscriber('/mask_result', String, self.mask_cb, queue_size=1)
        rospy.Subscriber('/button', String, self.button_cb, queue_size=1)
        rospy.Subscriber('/start_ui', String, self.ui_button_cb, queue_size=1)
        rospy.Subscriber('/phone_number', String, self.ph_no_ui_cb, queue_size=1)
        rospy.Subscriber('/capture_result', String, self.capture_cb, queue_size=1)
        rospy.Subscriber('/temperature_control_result', String, self.temp_cb, queue_size=1)
        rospy.Subscriber('/voltage_raw_', Int32, self.voltage_cb, queue_size=1)
        rospy.Subscriber('/speech_recognize_complete', String, self.speech_recognize_cb, queue_size=1)
        rospy.Subscriber('/menu_grab_result', String, self.detailed_menu_cb, queue_size=1)

        ###
        self.checkdb_pub = rospy.Publisher("checkdb_control", String, queue_size=1)
        rospy.Subscriber('/checkdb_control_result', String, self.checkdb_control_cb, queue_size=1)
        self.insertdb_pub = rospy.Publisher("/insertdb_control", String, queue_size=1)
        self.menu_pub = rospy.Publisher("/menu_grab_control", String, queue_size=1)

        self.hotword_result = ""
        self.person_result = ""
        self.face_result = ""
        self.temp_result = ""

        self.battery_percent = 100
        self.voltage_low = False

        self.pm_event = False
        self.arm_project = False
        self.sevabot = False
        self.mask_result = ""
        self.voice_result_number = ""
        self.button_click_result = ""
        self.voice_result_menu = ""
        self.button_pressed = ""
        self.button_result = ""
        self.voicebutton_response = False
        self.ph_no = ""
        self.capture_result = ""
        self.ui_process_started = False
        # added for guidance
        self.guidance_current_zone = 0
        self.guidance_process_seq = 0
        self.all_tour = False
        self.tour_started = False
        self.robot_current_position = 0
        self.robot_previous_position = 0
        self.nav_result = ""
        self.guidance_started = False
        self.mic_status = True
        self.start_process_joy = False
        self.faq_called = False
        #### joy
        self.navigating = False
        self.shutdown_cnt = 0
        self.restart_cnt = 0
        # rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)
        self.cmd_repub = rospy.Publisher("/diffbot_controller/cmd_vel", Twist, queue_size=1)
        self.shut_pub = rospy.Publisher("/shut", String, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1)
        self.sanitizer_pump_control = rospy.Publisher('/sanitizer_pump', Bool, queue_size=10)

        self.zone_second_position_reached = False
        # self.twitter_selected = False
        self.name_dict = {}

        self.confirmation_dict = {0: self.change_confirmation_z0, 1: self.change_confirmation_z1,
                                  2: self.change_confirmation_z2, 3: self.change_confirmation_z3,
                                  4: self.change_confirmation_z4,
                                  5: self.change_confirmation_z5, 6: self.change_confirmation_z6,
                                  7: self.change_confirmation_z7, 8: self.change_confirmation_z8,
                                  9: self.change_confirmation_z9, 10: self.change_confirmation_z10}

        self.zone_dict = {0: self.change_z0, 1: self.change_z1, 2: self.change_z2, 3: self.change_z3, 4: self.change_z4,
                          5: self.change_z5, 6: self.change_z6, 7: self.change_z7, 8: self.change_z8, 9: self.change_z9,
                          10: self.change_z10, 13: self.change_charge}
        self.zone_no_name_dict = {0: "home", 1: "tlZone", 2: "shZone", 3: "ceZone", 4: "vrZone", 5: "slZone",
                                  6: "mpZone", 7: "ccZone", 8: "hZone", 9: "feedback", 10: "twitter", 11: "out_dxc",
                                  12: "in_dxc", 13: "dock"}

        self.zone_gesture_dict = {0: "", 1: "lefthand", 2: "twohands", 3: "twohands", 4: "twohands",
                                  5: "twohands", 6: "righthand", 7: "twohands", 8: "twohands", 9: "twohands",
                                  10: "twohands"}

        self.zone_no_of_end_position_dict = {0: 1, 1: 2, 2: 1, 3: 1, 4: 1,
                                             5: 1, 6: 1, 7: 1, 8: 1, 9: 1, 10: 1, 11: 1, 12: 1}

        self.map_dict = {"0": self.home_page, "1": self.wait_for_person, "2": self.check_face, "3": self.check_mask,
                         "4": self.check_temp_sanitize, "5": self.registration_deciding1,
                         "6": self.phone_no_entry, "7": self.capture_image, "8": self.display_qr,
                         "9": self.guidance_entry,
                         "10": self.options_showing, "11": self.faq, "12": self.feedback_func1,
                         "13": self.guidance_menu,
                         "14": self.guidance_flow, "15": self.guidance_zones,
                         "16": self.feedback_func2, "17": self.feedback_func3, "18": self.guidance_confirmation,
                         "19": self.battery_charging,
                         "22": self.settings_page, "23": self.training, "24": self.menu_page}

        self.goals = {"home": {"x": 1.255, "y": 0.644, "z": 0.8891, "w": 0.4569},
                      "infrontofdoor": {"x": 7.2932, "y": 2.1012, "z": 0.3568, "w": 0.9341},
                      "tlZone": {"x": 10.1627, "y": 4.1445, "z": 0.4912, "w": 0.8710},
                      "tlZone_1": {"x": 15.3771, "y": 7.6973, "z": 0.9935, "w": -0.1129},
                      "twitter": {"x": 3.8870, "y": 2.2874, "z": 0.47110, "w": 0.8820},
                      # "touchtable": {"x": 18.4970340729, "y": 6.62538433075, "z": 0.340766950575, "w": 0.940147799761},
                      "shZone": {"x": 17.7059, "y": 8.8990, "z": 0.9999, "w": -0.0070},
                      # "arZone": {"x": 17.4944820404, "y": 8.54388237, "z": 0.88350626773, "w": 0.468419336579},
                      "ceZone": {"x": 17.7059, "y": 8.8990, "z": 0.02207, "w": 0.9997},
                      "vrZone": {"x": 20.9329, "y": 4.1785, "z": -0.0542, "w": 0.9985},
                      "slZone": {"x": 20.6249, "y": 1.7760, "z": -0.1257, "w": 0.9920},
                      "mpZone": {"x": 20.2109, "y": 3.8064, "z": 0.9997, "w": 0.0225},
                      "ccZone": {"x": 19.8335, "y": 3.4935, "z": -0.9584, "w": 0.2850},
                      "hZone": {"x": 9.7039, "y": 3.6610, "z": -0.0296, "w": 0.9995},
                      "feedback": {"x": 5.6329, "y": 0.9575, "z": 0.7095, "w": 0.7046},
                      "doorreturn": {"x": 9.7053, "y": 3.6770, "z": 0.9483, "w": -0.3172}
                      }

        self.phonenumber = ""
        self.formdb_control_result = []
        self.voice_result_feedback = ""
        self.voice_feedback_received = False
        self.feedback_question_timeout = 20  # 1minute timeout
        self.feedback_voice_timeout = 60  # 1 minute timeout
        self.feedback_submit_timeout = 60  # 1 minute timeout

        self.avg_percent = 0
        self.count_percent = 0

        self.questions = ["1. Overall, how was your experience in DXC",
                          "2. After the Visit, how inspired did you feel?",
                          "3. Which are the technologies you liked the most in DXC?",
                          "4. How informative were these technologies?",
                          "5. What are the Technologies that inspire you to adapt it in your workspace?",
                          "6. How much time did you spend inside DXC?",
                          "7. Overall, were you satisfied with the way technologies were showcased?",
                          "8. What other technologies would you like to see in the DXC ?"]
        self.choices = [["0", "1", "2", "3", "4", "5"], ["0", "1", "2", "3", "4", "5"],
                        ["Holographic", "Robot", "Command centre", "AR", "VR", "MR", "Touch table", "blockchain",
                         "Analytics", "Forecasting", "RPA", "Connected Worker", "Asset management", "Ai camera",
                         "Drone tech"], ["0", "1", "2", "3", "4", "5"],
                        ["Holographic", "Robot", "Command centre", "AR", "VR", "MR", "Touch table", "blockchain",
                         "Analytics", "Forecasting", "RPA", "Connected Worker", "Asset management", "Ai camera",
                         "Drone tech"], ["<10 Min", "10-30 Min", ">30 Min"], ["0", "1", "2", "3", "4", "5"]]
        self.answers = []
        self.answer = []

        self.normaltemp_low = 36
        self.normaltemp_high = 38
        self.hightemp = 39

    def ui_button_cb(self, data):
        self.ui_process_started = True

    def train_cb(self, data):
        if data.data == "done":
            self.train_result = "completed"
        if data.data == "failed":
            self.train_result = "failed"

    def mic_cb(self, data):
        if data.data == "false":
            self.mic_status = False

    def speech_recognize_cb(self, data):
        self.person_speaked = True

    def interpolate(self, x1, x2, y1, y2, x):
        output = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        return output

    def voltage_cb(self, data):

        percentage = self.interpolate(540, 750, 0, 100, data.data)
        if percentage < 0:
            percentage = 0
        self.count_percent += 1
        if self.count_percent < 10:
            if self.count_percent == 1:
                self.avg_percent = percentage
            self.avg_percent = (self.avg_percent + percentage) / 2
            return
        self.count_percent = 0
        percentage = int(self.avg_percent)
        if percentage > 0:
            self.battery_percent = 10
        if percentage > 10:
            self.battery_percent = 20
        if percentage > 20:
            self.battery_percent = 40
        if percentage > 40:
            self.battery_percent = 60
        if percentage > 60:
            self.battery_percent = 80
        if percentage > 80:
            self.battery_percent = 100

        self.battery.publish(str(self.battery_percent))
        if percentage <= 10:
            self.voltage_low = True
        if percentage >= 40:
            self.voltage_low = False

    def cmd_cb(self, data):
        if not self.navigating:
            self.cmd_repub.publish(data.data)

    def holding_restart_check(self, timeout=3):
        cnt = 0

        while not rospy.is_shutdown() and self.restart_pressed:
            time.sleep(0.1)
            cnt += 1
            if cnt > int(timeout * 10):
                self.restart_pressed = False
                self.exec_audio_block("button_click")
                self.shut_pub.publish("restart")
                return

    def holding_shutdown_check(self, timeout=3):
        cnt = 0

        while not rospy.is_shutdown() and self.shutdown_pressed:
            time.sleep(0.1)
            cnt += 1
            if cnt > int(timeout * 10):
                self.shutdown_pressed = False
                self.exec_audio_block("button_click")
                self.shut_pub.publish("shutdown")
                return

    def joy_cb(self, data):

        if data.buttons[0] == 1:
            print("button X")

            self.exec_gesture("posture_up")

        if data.buttons[3] == 1:
            print("button Y")
            self.sanitizer_pump_control.publish("true")

        if data.buttons[8] == 1:
            print("button Back")
            self.shutdown_pressed = True
            shut_thread = threading.Thread(target=self.holding_shutdown_check)
            shut_thread.start()
        else:
            self.shutdown_pressed = False

        if data.buttons[9] == 1:
            print("button Start")
            self.restart_pressed = True
            restart_thread = threading.Thread(target=self.holding_restart_check)
            restart_thread.start()
        else:
            self.restart_pressed = False

        if data.buttons[5] == 1:
            print("button RB")
            self.exec_gesture("large_talk1")

        if data.buttons[1] == 1:
            print("button A")
            self.show_video("nationalanthem.mp4")

        if data.buttons[2] == 1:
            print("button B")
            self.close_video()

        if data.buttons[4] == 1:
            print("button LB")

        if data.buttons[7] == 1:
            print("button RT")
            self.nav_ended = "done"

        if data.buttons[6] == 1:
            print("button LT")
            self.exec_gesture("down_salute")

    def nav_cb(self, data):
        self.nav_result = data.data

    def ph_no_ui_cb(self, data):
        dict1 = json.loads(data.data)
        self.ph_no = dict1["phone_number"].replace(" ", "")

    def save_dict(self):

        with open('/home/asimov/IRA_V2_ws/src/data.pickle', 'wb') as handle:
            pickle.dump(self.name_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def exec_audio_block(self, name):
        subprocess.call('mpg123 /home/asimov/IRA_V2_ws/src/audio/railway/' + name + '.mp3', shell=True)

    def exec_audio_block_gesture(self, name):
        duration = int(self.check_audio_timing(name) - 4)  # normal duration - 4 seconds
        if duration > 4:
            self.long_speech_gesture.publish(str(duration))
        subprocess.call('mpg123 /home/asimov/IRA_V2_ws/src/audio/railway/' + name + '.mp3', shell=True)
        # self.long_speech_gesture.publish("stop")

    def show_video(self, name):
        subprocess.call('sh /home/asimov/IRA_V2_ws/src/saya_bringup/script/show_video.sh ' + name, shell=True)

    def close_video(self):
        subprocess.call('sh /home/asimov/IRA_V2_ws/src/saya_bringup/script/close_video.sh', shell=True)

    def exec_show_sh(self, name):
        subprocess.call('sh /home/asimov/IRA_V2_ws/src/saya_bringup/script/show_image.sh ' + name, shell=True)

    def exec_close_sh(self):
        subprocess.call('sh /home/asimov/IRA_V2_ws/src/saya_bringup/script/close_image.sh', shell=True)

    def exec_gesture(self, name):
        subprocess.call("roslaunch eva_arm_controller " + name + ".launch", shell=True)
        print(name)

    def check_audio_timing(self, name):

        fname = "/home/asimov/IRA_V2_ws/src/audio/railway/" + name + ".mp3"
        duration = 0
        audio = MP3(fname)
        duration = int(audio.info.length)
        audio.delete()
        return duration

    def exec_audio_live(self, text):
        try:
            python3_command = "/home/asimov/IRA_V2_ws/src/saya_communication/saya_text_speech/scripts/google_speech_python.py"  # launch your python2 script using bash
            py2output = subprocess.check_output(["python3", python3_command, '-t', text])
        except:
            pass
        return

    def load_dict(self):

        with open('/home/asimov/IRA_V2_ws/src/data.pickle', 'rb') as handle:
            self.name_dict = pickle.load(handle)

    def hotword_cb(self, data):
        self.hotword_result = data.data

    def execute_hand_raises(self, name):
        self.exec_gesture(name + "_up")
        time.sleep(0.1)
        self.exec_gesture(name + "_down")

    def start_zoom(self):
        t1 = threading.Thread(target=self.zoom_open)
        t1.start()

    def zoom_open(self):
        command = "zoom"
        subprocess.call(command, shell=True)

    def zoom_close(self):
        command = "pkill zoom"
        subprocess.call(command, shell=True)

    def capture_cb(self, data):
        self.capture_result = data.data

    def entry_cb(self):
        pass

    ###
    def checkdb_control_cb(self, msg):
        print("phonenumbersub db", msg)
        self.formdb_control_result = json.loads(msg.data)["result"]

    def person_cb(self, data):
        self.person_result = data.data
        print(data.data)

    def face_cb(self, data):
        self.face_result = data.data
        print(data.data)

    def mask_cb(self, data):
        self.mask_result = data.data
        print(data.data)

    def temp_cb(self, data):
        self.temp_result = data.data
        print(data.data)

    def checkentry_cb(self, data):
        self.face_name = data.data
        print(data.data)

    def button_cb(self, data):
        dict1 = json.loads(data.data)
        print(dict1)
        self.exec_audio_block("button_click")
        self.button_pressed = dict1["button"]
        ###
        if (self.button_pressed == "next" and dict1["screen_id"] == "feedBack"):
            self.answer = dict1["feed_back"]
        if (self.button_pressed == "next" and dict1["screen_id"] == "voiceFeedBack"):
            self.answer = [dict1["feedback_voice"]]
        if self.button_pressed == "submit" and dict1["screen_id"] == "feedBackSubmit":
            self.answer = [dict1["content"]]
        if dict1["screen_id"] == "feedBackSubmit":
            if "keypad" in dict1:
                print(dict1["keypad"])
                if len(dict1["keypad"]) == 10:
                    self.phonenumber = dict1["keypad"].replace(" ", "")
                    print("cb phone", self.phonenumber)

    def button_control_cb(self, data):

        dict1 = json.loads(data.data)
        print(dict1)

        self.button_result = dict1["button"]
        ###
        if (self.button_result == "next" and dict1["screen_id"] == "feedBack"):
            self.answer = dict1["feed_back"]
            self.voicebutton_response = True
        if (self.button_result == "next" and dict1["screen_id"] == "voiceFeedBack"):
            self.answer = [dict1["feedback_voice"]]
            self.voicebutton_response = True
        if self.button_result == "submit" and dict1["screen_id"] == "feedBackSubmit":
            self.answer = [dict1["content"]]
            self.voicebutton_response = True
        print(self.answer, "asnwer in voice button callback")

        # if dict1["screen_id"] == "feedBackSubmit":
        #     if "keypad" in dict1:
        #         print(dict1["keypad"])
        #         if len(dict1["keypad"]) == 10:
        #             self.phonenumber = dict1["keypad"].replace(" ", "")
        #             print("cb phone", self.phonenumber)

    def voice_cb(self, data):

        dict1 = json.loads(data.data)
        if dict1["type"] == "phone_number":
            self.voice_result_number = dict1["result"].replace(" ", "")

        if dict1["type"] == "menu":
            self.voice_result_menu = dict1["result"].lower()
            print("voice result", self.voice_result_menu)

        ###
        if dict1["type"] == "feedback":
            self.voice_result_feedback = self.voice_result_feedback + " " + dict1["result"]
            print("voice result", self.voice_result_menu)
            self.voice_feedback_received = True

    def menu_cb(self, data):
        self.voice_result_menu = data.data


    def detailed_menu_cb(self, data):
        temp = json.loads(data.data)
        temp_items_selected = temp["items"]
        temp_items_count = temp["counts"]
        self.total_bill = temp["total"]
        self.text_menu = ""
        for i in range(len(temp_items_selected)):
            self.text_menu += temp_items_selected[i] + "-" + str(temp_items_count[i]) + " "

        self.text_menu += " " + "Rs." + str(self.total_bill)

        print(self.text_menu)


    def start_training(self):
        self.train_result = ""
        self.train_control.publish("start")

    # get the phone number through voice
    def start_get_no_voice(self, timeout=20):
        self.exec_audio_block("button_click")

        print("ponenumber vuice pub to ui")
        self.voice_result_number = ""
        ###
        self.voice_result_feedback = ""
        self.voice_result_menu = ""
        dict1 = {"type": "phone_number", "timeout": timeout}
        self.voice_input.publish(json.dumps(dict1))
        time.sleep(1)

    # get the menu selection through voice
    def start_get_menu_voice(self, context=[], timeout=40):

        print("menu vuice pub to ui")

        self.voice_result_menu = ""
        ###

        dict1 = {"type": "menu", "timeout": timeout, "contexts": context}
        self.voice_input.publish(json.dumps(dict1))
        # self.menu_input.publish("start")

    def stop_get_no_voice(self):
        print(" stop menu voice ")
        self.voice_result_number = ""
        self.voice_result_menu = ""
        ###
        self.voice_result_feedback = ""
        dict1 = {"type": "stop"}
        self.voice_input.publish(json.dumps(dict1))

    def stop_get_menu_voice(self):
        print(" stop menu voice ")
        self.voice_result_number = ""
        self.voice_result_menu = ""
        ###
        self.voice_result_feedback = ""
        # self.menu_input.publish("stop")
        dict1 = {"type": "stop"}
        self.voice_input.publish(json.dumps(dict1))

    ###
    def start_get_feedback_voice(self, timeout=60):
        self.exec_audio_block("button_click")
        print("ponenumber vuice pub to ui")
        self.voice_result_number = ""
        self.voice_result_menu = ""
        self.voice_result_feedback = ""
        dict1 = {"type": "feedback", "timeout": timeout}
        self.voice_input.publish(json.dumps(dict1))
        time.sleep(1)

    ###
    def stop_get_feedback_voice(self):
        print(" stop menu voice ")
        self.voice_result_number = ""
        self.voice_result_menu = ""
        self.voice_result_feedback = ""
        dict1 = {"type": "stop"}
        self.voice_input.publish(json.dumps(dict1))

    # get data of visitor from database using number
    def get_data_from_database(self, number):
        pass

    def movebase_publish(self, goal_x, goal_y, goal_z, goal_w):
        global goal_publish

        goalId = 0
        goalMsg = PoseStamped()
        goalMsg.header.frame_id = "map"
        goalMsg.pose.orientation.z = goal_z
        goalMsg.pose.orientation.w = goal_w
        # Publish the first goal

        goalMsg.header.stamp = rospy.Time.now()
        goalMsg.pose.position.x = goal_x
        goalMsg.pose.position.y = goal_y
        self.goal_publish.publish(goalMsg)
        print("goal published", goal_x, goal_y)

    # check person is regstered or not ...
    # key = number or name
    # type = number or name
    def check_person_registration(self, key, type="number"):
        pass

    def exec_audio_by_name(self, name_audio):
        name_audio = name_audio.replace("Ms.", "Miss")
        name_audio = name_audio.replace("Mrs.", "Misses")
        name_audio = name_audio.replace("Mr.", "Mister")
        name_audio = name_audio.replace("Mrs.", "Misses")
        name_audio = name_audio.replace("_", " ")
        self.exec_audio_live("Hello " + name_audio)

    def start_hotword(self):
        self.hotword_result = ""
        self.hotword.publish("start")

    def stop_hotword(self):
        self.hotword_result = ""
        self.hotword.publish("stop")

    def start_faq(self):

        self.faq_pub.publish("true")
        print("start_faq")

    def stop_faq(self):
        self.faq_pub.publish("false")
        print("stop")

    def start_person_check(self):
        self.person_result = ""
        self.person_control.publish("start")
        print("person_check")

    def stop_person_check(self):
        self.person_control.publish("stop")

    def start_face_check(self):
        self.face_result = ""
        self.face_control.publish("start")
        print("face_check")

    def stop_face_check(self):
        self.face_control.publish("stop")

    def start_mask_check(self):
        self.mask_control.publish("start")
        self.mask_result = ""
        print("mask check")

    def stop_mask_check(self):
        self.mask_control.publish("stop")

    def start_formcheck(self):
        self.face_name = ""
        self.check_entry.publish(self.ph_no)

    def stop_formcheck(self):
        self.check_entry.publish("stop")

    def start_temp_check(self):
        self.temp_control.publish("start")
        self.temp_result = ""

        print("temp check")

    def stop_temp_check(self):
        self.temp_control.publish("stop")

    def start_capture(self):
        self.capture_result = ""
        self.capture_pub.publish(self.ph_no)

    def stop_capture(self):
        self.capture_result = ""
        self.capture_pub.publish("stop")

    def training(self):
        self.change_training()
        self.start_training()
        self.exec_audio_live("please wait till training completes")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.train_result == "completed":
                self.exec_audio_live("Training completed succesfully")
                self.shut_pub.publish("restart")
            if self.train_result == "failed":
                self.exec_audio_live("Training failed")
                self.current_process = 1
                return

    def pm_page(self):
        self.change_picture1("pm")
        t2 = threading.Thread(target=self.exec_gesture, args=("namaste",))
        t2.start()
        time.sleep(1)
        self.exec_audio_block("pm")
        cnt = 0
        while not rospy.is_shutdown() and self.pm_event:
            time.sleep(0.5)
        t3 = threading.Thread(target=self.exec_gesture, args=("right_hand_guide",))
        t3.start()
        self.exec_audio_block("pm2")
        time.sleep(1)
        self.current_process = 0
        t3.join()

    def settings_page(self):
        self.change_page("systemOptions")
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "shutdown":
                self.shut_pub.publish("shutdown")
                self.button_pressed = ""
                self.current_process = 0
                return
            if self.button_pressed == "restart":
                self.shut_pub.publish("restart")
                self.button_pressed = ""
                self.current_process = 0
                return
            if self.button_pressed == "charge":
                self.button_pressed = ""
                self.current_process = 19
                return
            if self.button_pressed == "sanitizer":
                self.sanitizer_pump_control.publish(True)
                time.sleep(1)
                self.sanitizer_pump_control.publish(False)
                self.button_pressed = ""

            if self.button_pressed == "back":
                self.button_pressed = ""
                self.current_process = 1
                return

            if self.button_pressed == "train":
                self.button_pressed = ""
                self.current_process = 23
                return

    def battery_charging(self):
        self.change_page("roboBatteryStatus", ["waste"])
        self.exec_audio_live("Going to charge")
        self.guidance_current_zone = 0
        self.robot_goal_position = 0

        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.robot_current_position != 0:
                # self.twitter_selected = False
                res = self.navigate()

                if res == "done":
                    break
            else:
                break
        self.change_page("roboBatteryStatus", ["Charging.. Please say HI RACER to interrupt"])
        self.start_hotword()
        while not rospy.is_shutdown():
            time.sleep(0.5)

            if self.hotword_result == "wake":
                #                self.exec_audio_live("I am on charging. Please give me some time")
                if self.battery_percent >= 40:
                    self.exec_audio_live("please say ok stop to exit charging instantly")
                    self.change_page("roboBatteryStatus", ["please say OK STOP to exit charging instantly"])
                else:
                    self.exec_audio_live("I am on charging. Please give me some time")
                self.start_hotword()

            if self.hotword_result == "quit":
                self.exec_audio_live("Please  wait i am getting ready")
                self.current_process = 1

                return

            if self.battery_percent >= 80:
                self.current_process = 1
                return

    def home_page(self):
        self.ui_process_started = False
        self.change_page("homePage")
        self.start_process_joy = False
        while not rospy.is_shutdown():
            time.sleep(1)
            self.change_page("homePage")
            if self.ui_process_started:
                self.current_process = 1
                return

    def wait_for_person(self):
        self.start_person_check()
        self.change_page("covidScreening")
        self.exec_audio_block_gesture("facedetection")

        self.start_hotword()
        self.button_pressed = ""
        print("1")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.hotword_result == "wake":

                self.process_after_faq = 1
                self.current_process = 11
                self.stop_person_check()
                self.stop_hotword()
                return
            elif self.person_result != "":
                print("3")
                self.current_process = 10
                self.stop_person_check()
                self.stop_hotword()
                return
            if self.button_pressed == "refresh":
                self.stop_hotword()
                self.stop_person_check()
                self.current_process = 0
                return
            if self.button_pressed == "settings":
                self.stop_hotword()
                self.stop_person_check()
                self.current_process = 22
                return

        self.stop_hotword()
        self.stop_person_check()

    def check_face(self):
        self.face_name = ""
        self.ph_no = ""
        self.start_face_check()
        self.start_hotword()
        self.face_recognized = False
        print("init")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            print("start")
            if self.hotword_result == "wake":
                print("wake")
                self.process_after_faq = 1
                self.current_process = 11
                self.stop_face_check()
                self.stop_hotword()
                return

            if self.face_result == "":
                print("empty")
                pass
            if self.button_pressed == "refresh":
                self.stop_hotword()
                self.stop_face_check()
                self.current_process = 0
                return
            if self.button_pressed == "settings":
                self.stop_hotword()
                self.stop_face_check()
                self.current_process = 22
                return

            if self.face_result == "Unknown":
                print("unknown.fac")
                self.current_process += 1
                self.stop_hotword()
                self.stop_face_check()
                return
            elif self.face_result == "missed":
                print("misssed")
                self.current_process = 1
                self.stop_hotword()
                self.stop_face_check()
                return
            else:
                if self.face_result != "":
                    print("4", "known", self.face_result)
                    self.current_process += 1
                    self.face_recognized = True
                    self.face_result.replace("_", " ")
                    self.face_name = self.face_result
                    print("jhh", self.face_name)
                    self.stop_hotword()
                    self.stop_face_check()
                    if "jain" in self.face_name:
                        self.change_picture1("welcome_tarun")
                        self.exec_audio_block_gesture("welcome_tarun")
                        self.change_page("covidScreening")
                    else:
                        self.exec_audio_by_name(self.face_name)
                    return

        self.stop_hotword()
        self.stop_face_check()

    def check_mask(self):
        self.start_mask_check()
        self.start_hotword()
        # self.exec_audio("")
        print("startmask")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.hotword_result == "wake":
                print("mask wakeword")
                self.process_after_faq = 1
                self.current_process = 11
                self.stop_mask_check()
                self.stop_hotword()
                return
            if self.button_pressed == "refresh":
                self.stop_hotword()
                self.stop_mask_check()
                self.current_process = 0
                return
            if self.button_pressed == "settings":
                self.stop_hotword()
                self.stop_mask_check()
                self.current_process = 22
                return

            if self.mask_result == "":
                print("maskempty")
                pass

            if self.mask_result == "missed":
                print("missed")
                self.current_process = 1
                self.stop_hotword()
                self.stop_mask_check()
                return
            else:
                if self.mask_result != "":
                    print("mask", self.mask_result)
                    print("worn")
                    self.current_process += 1
                    self.stop_hotword()
                    self.stop_mask_check()
                    dict = {"temperature": "Measuring In ", "mask_status": self.mask_result}
                    self.update_texts("covidScreening", dict)
                    return

        self.stop_hotword()
        self.stop_mask_check()

    def check_temp_sanitize(self):
        self.start_temp_check()
        while not rospy.is_shutdown():
            time.sleep(0.5)
            dict = {"temperature": "Measuring In", "mask_status": self.mask_result}
            self.update_texts("covidScreening", dict)
            if self.hotword_result == "wake":
                self.process_after_faq = 1
                self.current_process = 11
                self.stop_temp_check()
                self.stop_hotword()
                return
            if self.temp_result == "":
                pass
            elif self.temp_result == "time_out":
                self.current_process = 1
                return
            elif int(self.temp_result) >= self.normaltemp_low and int(self.temp_result) <= self.normaltemp_high:
                self.current_process += 1
                dict = {"temperature": self.temp_result, "mask_status": self.mask_result}
                self.update_texts("covidScreening", dict)
                time.sleep(3)
                return
            else:
                self.current_process = 1
                if int(self.temp_result) > self.hightemp:
                    dict = {"temperature": self.temp_result, "mask_status": self.mask_result}
                    self.update_texts("covidScreening", dict)
                    time.sleep(3)
                return
        self.stop_temp_check()

    def registration_deciding1(self):
        print("facerecognsied", self.face_recognized)
        if self.face_recognized:
            self.current_process = 9
        else:
            self.current_process += 1

    def phone_no_entry(self):
        self.ph_no = ""
        self.button_pressed = ""

        self.change_page("inputNumber")

        dict = {"phone_number": ""}
        self.update_texts("inputNumber", dict)

        self.exec_audio_block_gesture("phno")
        self.voice_result_menu = ""

        self.start_get_no_voice(40)
        self.mic_status = True

        cnt = 0
        while not rospy.is_shutdown():
            time.sleep(0.5)
            cnt += 1

            if cnt > 50:  # 25 seconds
                self.ph_no = ""
                self.stop_get_no_voice()
                self.stop_get_menu_voice()
                self.current_process = 1
                print("back pressed")
                return

            if self.voice_result_number != "":
                # second robot
                cnt = 0

                dict = {"phone_number": self.voice_result_number}
                self.update_texts("inputNumber", dict)

                if len(self.voice_result_number) > 9:
                    # self.change_screen_ph_no(["Please click the mobile number field to change your number"])
                    self.stop_get_no_voice()
                    time.sleep(0.5)
                    self.start_get_menu_voice(["ok submit", "ok back"])
                self.voice_result_number = ""

            if not self.mic_status:
                cnt = 0

                self.stop_get_no_voice()
                self.start_get_menu_voice(["ok submit", "ok back"])
                self.mic_status = True
                self.voice_result_number = ""

            # if self.voice_result_menu == "ok submit" or self.button_pressed == "back":
            if self.voice_result_menu == "ok submit":
                dict = json.dumps({"screen_id": "inputNumber", "button": "submit"})
                self.button_control_pub.publish(dict)
                self.stop_get_no_voice()
                self.stop_get_menu_voice()
                self.voice_result_number = ""
                self.voice_result_menu = ""

            if len(self.ph_no) > 9:
                print(self.ph_no, type(self.ph_no))
                #                if self.ph_no in self.name_dict.keys():
                #                    self.face_name = self.name_dict[self.ph_no]
                #                    self.current_process = 9
                #                    # self.exec_audio_block("alreadyregistered")
                #                    self.exec_audio_by_name(self.face_name)
                #                else:
                #                    self.current_process += 1
                self.formdb_control_result = []
                if not self.formdb_control_result:
                    dict = {"query": ["NAME"], "phone_number": self.ph_no,
                            "control": "start"}
                    self.checkdb_pub.publish(json.dumps(dict, sort_keys=False))
                while not rospy.is_shutdown():
                    time.sleep(0.5)
                    if self.formdb_control_result:
                        print("db", self.formdb_control_result)
                        if self.formdb_control_result[0] != "notfound":
                            self.face_name = self.formdb_control_result[0]
                            self.current_process = 9
                            self.formdb_control_result = []
                            # self.exec_audio_by_name(self.face_name)
                            break
                        else:
                            self.formdb_control_result = []
                            self.current_process += 1
                            print(self.ph_no, "not found phno")
                            break
                return
            # second robot
            # if self.button_pressed == "back" or self.voice_result_menu == "ok back":
            if self.button_pressed == "back" or self.voice_result_menu == "ok back":
                self.ph_no = ""
                self.stop_get_no_voice()
                self.stop_get_menu_voice()
                self.current_process = 1
                print("back pressed")
                return
        self.start_get_no_voice()

    def capture_image(self):
        self.change_page("userPhotoCapture")
        self.exec_audio_block_gesture("capture")
        print("capture1", self.ph_no)
        self.start_capture()

        ### do in second robot
        self.capture_result = ""
        self.voice_result_menu = ""
        self.button_pressed = ""
        print("capture2", self.ph_no)
        self.start_get_menu_voice(["ok capture"])

        cnt = 0
        while not rospy.is_shutdown():
            time.sleep(0.5)
            cnt += 1

            if cnt > 50:  # 25 secs
                self.stop_get_menu_voice()
                self.current_process = 1
                return

            if self.button_pressed == "capture" or self.voice_result_menu == "ok capture":
                time.sleep(2)
                cnt = 0
                self.capture_button.publish("capture")

            if self.capture_result != "":
                self.stop_get_menu_voice()
                self.current_process += 1
                return

        self.stop_capture()
        self.stop_get_menu_voice()

    def display_qr(self):
        self.face_name = ""
        self.change_page("qrCodePage")
        self.animation_change("qrCodePage", ["qr"], "jpg")
        self.exec_audio_block_gesture("qr")
        self.start_formcheck()
        self.button_pressed = ""
        self.start_get_menu_voice(["ok back", "ok cancel"])
        start_time = rospy.Time.now().secs

        cnt = 0
        while not rospy.is_shutdown():

            cnt += 1

            current_time = rospy.Time.now().secs
            print(current_time - start_time)
            time.sleep(0.5)
            if self.face_name != "":
                if self.face_name != "notfound":
                    #                    self.name_dict.update({self.ph_no: self.face_name})
                    #                    self.save_dict()
                    self.stop_formcheck()
                    self.stop_get_menu_voice()
                    self.exec_audio_block("registration_complete")
                    # self.exec_audio_by_name(self.face_name)
                    self.current_process += 1
                    return
            if cnt >= 600 or self.face_name == "notfound":
                self.stop_formcheck()
                self.stop_get_menu_voice()
                self.current_process = 1  # return to covid screening
                return

            # if self.button_pressed == "back" or self.voice_result_menu == "ok back":
            if self.button_pressed == "back" or self.voice_result_menu == "ok back":
                self.current_process = 7
                self.stop_get_menu_voice()
                self.stop_formcheck()
                print("back pressed")
                return

            # if self.button_pressed == "cancel" or self.voice_result_menu == "ok cancel":
            if self.button_pressed == "cancel" or self.voice_result_menu == "ok cancel":
                self.current_process = 1
                self.stop_get_menu_voice()
                self.stop_formcheck()
                print("back pressed")
                return
        self.stop_formcheck()
        self.stop_get_menu_voice()

    def guidance_entry(self):
        # self.ph_no = ""
        # dict = {"phone_number": ""}
        # self.update_texts("inputNumber", dict)
        self.change_page("guidance", [self.face_name])

        self.exec_audio_by_name(self.face_name)
        self.exec_audio_block_gesture("guidance_entry")
        self.button_pressed = ""
        self.start_get_menu_voice(["ok accept", "ok skip"])

        cnt = 0

        while not rospy.is_shutdown():
            time.sleep(0.5)
            cnt += 1

            if cnt == 50:  # 25 secs
                self.stop_get_menu_voice()
                self.current_process = 1
                return

            print(self.voice_result_menu)
            # if "skipAcceptGuidance" in self.button_pressed or self.voice_result_menu == "ok skip" or self.voice_result_menu == "skip guidance":
            if "skip" in self.button_pressed or self.voice_result_menu == "ok skip" or self.voice_result_menu == "skip guidance":
                self.current_process += 1
                self.stop_get_menu_voice()
                print("skip pressed")
                return
            # if "acceptGuidance" in self.button_pressed or self.voice_result_menu == "ok accept" or self.voice_result_menu == "accept guidance":
            if "accept" in self.button_pressed or self.voice_result_menu == "ok accept" or self.voice_result_menu == "accept guidance":
                self.current_process = 14  # create accept guidance process
                self.stop_get_menu_voice()
                self.tour_started = True
                self.guidance_current_zone = 0
                self.all_tour = True
                # self.twitter_selected = True
                print("accept pressed")
                return
        self.stop_get_menu_voice()

    def guidance_menu(self):
        self.all_tour = False
        self.change_page("guidanceMenu")
        self.exec_audio_block_gesture("guidance_menu")

        if self.zone_no_of_end_position_dict[self.robot_current_position] > 1:
            if not self.zone_second_position_reached:
                self.exec_audio_block_gesture("second_position_waiting")
                name = self.zone_no_name_dict[self.robot_current_position] + "_1"
                self.navigate_with_position(name)
                self.zone_second_position_reached = True

        self.start_hotword()
        self.start_get_menu_voice(["ok back", "smart helmet zone", "AR zone", "timeline zone", "self learning zone",
                                   "command and control centre zone", "central exploratory zone",
                                   "holographic zone", "VR and MR zone",
                                   "multitouch phygital zone"])
        self.button_pressed = ""

        # self.exec_audio("")
        cnt = 0
        while not rospy.is_shutdown():
            print(self.voice_result_menu)
            time.sleep(0.5)
            cnt += 1

            if cnt == 50:  # 25 secs
                self.stop_get_menu_voice()

            if self.hotword_result == "wake":
                self.process_after_faq = 13
                self.current_process = 11
                self.stop_hotword()
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "back" or self.voice_result_menu == "ok back":
            if cnt > 600 or self.button_pressed == "back" or self.voice_result_menu == "ok back":
                self.stop_hotword()
                self.current_process = 10
                self.stop_get_menu_voice()
                print("back pressed")
                return

            ### do in second robot
            # if self.button_pressed == "shZone" or self.voice_result_menu == "smart-helmet" or self.voice_result_menu == "AR-zone":

            if self.button_pressed == "tlZone" or self.voice_result_menu == "timeline zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 1
                self.stop_get_menu_voice()
                return

            if self.button_pressed == "shZone" or self.voice_result_menu == "smart helmet zone" or self.voice_result_menu == "AR zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 2
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "ceZone" or self.voice_result_menu == "exploratory":
            if self.button_pressed == "ceZone" or self.voice_result_menu == "central exploratory zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 3
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "vmZone" or self.voice_result_menu == "V-R-and-M-R":
            if self.button_pressed == "vmZone" or self.voice_result_menu == "VR and MR zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 4
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "slZone" or self.voice_result_menu == "self-learning":
            if self.button_pressed == "slZone" or self.voice_result_menu == "self learning zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 5
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "mpZone" or self.voice_result_menu == "multi-touch":
            if self.button_pressed == "mpZone" or self.voice_result_menu == "multitouch phygital zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 6
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "ccZone" or self.voice_result_menu == "command-center":
            if self.button_pressed == "ccZone" or self.voice_result_menu == "command and control centre zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 7
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "hZone" or self.voice_result_menu == "holographic":
            if self.button_pressed == "hZone" or self.voice_result_menu == "holographic zone":
                self.stop_hotword()
                self.current_process = 14
                self.guidance_current_zone = 8
                self.stop_get_menu_voice()
                return

        self.stop_hotword()
        self.stop_get_menu_voice()

    # 10
    def options_showing(self):
        self.change_page("initialoption")
        self.exec_audio_block_gesture("options")
        self.start_hotword()
        # self.start_get_menu_voice(["ok back", "chatbot", "guidance", "feedback", "video call", "battery charge"])
        self.button_pressed = ""

        cnt = 0
        while not rospy.is_shutdown():
            time.sleep(0.5)

            if self.hotword_result == "wake":
                self.process_after_faq = 10
                self.current_process = 11
                self.stop_hotword()
                self.stop_get_menu_voice()
                return

            # if self.button_pressed == "back" or self.voice_result_menu == "ok back":
            if self.button_pressed == "back" or self.voice_result_menu == "ok back":
                self.stop_hotword()
                self.current_process = 0
                self.stop_get_menu_voice()
                print("back pressed")
                return
            # if self.button_pressed == "guidance" or self.voice_result_menu == "guidance":
            if self.button_pressed == "restaurant" or self.voice_result_menu == "restaurant":
                self.stop_hotword()
                self.current_process = 24
                self.stop_get_menu_voice()
                self.exec_audio_block_gesture('restaurant')
                self.nav_joy()
                print("guidance pressed")
                return

            # if self.button_pressed == "feedback" or self.voice_result_menu == "feedback":
            if self.button_pressed == "tourism" or self.voice_result_menu == "tourism":
                self.stop_hotword()
                self.current_process = 11
                self.process_after_faq = 10
                self.stop_get_menu_voice()
                self.exec_audio_block_gesture("tourism")
                return

            if self.button_pressed == "station" or self.voice_result_menu == "station":
                self.stop_hotword()
                self.exec_audio_block_gesture("station")
                self.start_hotword()
                self.button_pressed = ""
                # return
            # if self.button_pressed == "videoCall" or self.voice_result_menu == "video-call":
            if self.button_pressed == "train" or self.voice_result_menu == "train":
                self.stop_hotword()
                self.current_process = 31
                self.stop_get_menu_voice()
                return

        self.stop_hotword()
        self.stop_get_menu_voice()

    # 11
    def faq(self):
        self.button_pressed = ""
        self.change_page("faqPage")
        self.animation_change("faqPage", ["faq"], "mp4")

        self.start_hotword()
        self.start_faq()
        self.faq_called = True
        cnt = 0
        while not rospy.is_shutdown():
            time.sleep(0.5)
            cnt += 1

            if self.person_speaked:
                cnt = 0
                self.person_speaked = False

            if cnt > 50:  # i.e 25 seconds
                self.button_pressed = ""
                self.stop_hotword()
                self.stop_faq()
                self.current_process = self.process_after_faq
                return

            if self.hotword_result == "wake":
                self.start_hotword()

            if self.hotword_result == "quit" or self.button_pressed == "back":
                self.button_pressed = ""
                self.stop_hotword()
                self.stop_faq()
                self.current_process = self.process_after_faq
                return

        self.stop_hotword()
        self.stop_faq()

    # 24

    def menu_page(self):
        self.change_page("menuPage")
        self.exec_audio_block_gesture("menu")
        self.start_hotword()
        # self.start_get_menu_voice(["ok back", "chatbot", "guidance", "feedback", "video call", "battery charge"])
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.hotword_result == "wake":
                self.process_after_faq = 24
                self.current_process = 11
                self.stop_hotword()
                self.stop_get_menu_voice()
                return
            if self.button_pressed == "back":
                self.stop_hotword()
                self.current_process = 10
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "quickmenu":
                self.stop_hotword()
                self.current_process = 25
                self.stop_get_menu_voice()
                print("back pressed")
                return
            if self.button_pressed == "detailedmenu":
                self.stop_hotword()
                self.current_process = 32
                self.stop_get_menu_voice()
                print("back pressed")
                return

    # 25
    def quick_menu(self):
        self.text_menu = ""
        self.change_page("guidanceMenu")
        self.exec_audio_block_gesture("quick")
        self.start_hotword()
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.hotword_result == "wake":
                self.process_after_faq = 25
                self.current_process = 11
                self.stop_hotword()
                self.stop_get_menu_voice()
                return
            if self.button_pressed == "back":
                self.stop_hotword()
                self.current_process = 24
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "tea":
                self.stop_hotword()
                self.current_process = 26
                self.text_menu = "Tea Rs 20"
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "pavbhaji":
                self.stop_hotword()
                self.current_process = 26
                self.stop_get_menu_voice()
                self.text_menu = "Pavbhaji Rs 40"
                print("back pressed")
                return

            if self.button_pressed == "samosachat":
                self.stop_hotword()
                self.current_process = 26
                self.text_menu = "Samosachat Rs 30"
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "samosa":
                self.stop_hotword()
                self.current_process = 26
                self.text_menu = "Samosa Rs 25"
                self.stop_get_menu_voice()
                print("back pressed")
                return

    #32
    def detail_menu(self):
        self.text_menu = ""
        self.change_page("qrCodebillPage", ["कृपया क्यूआर कोड को स्कैन करं"])
        self.animation_change("qrCodePage", ["menu"], "jpg")
        self.exec_audio_block_gesture("bill")
        self.button_pressed = ""
        self.menu_pub.publish("start")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "back":
                self.stop_hotword()
                self.current_process = 24
                self.menu_pub.publish("stop")
                self.stop_get_menu_voice()
                print("back pressed")
                return

#             if self.button_pressed == "confirm":
#                 self.stop_hotword()
#                 self.current_process = 28
#                 self.nav_joy()
#                 self.stop_get_menu_voice()
#                 print("back pressed")
#                 return
            if self.text_menu != "":
                self.current_process = 26
                return



    # 26
    def confirm(self):
        self.change_page("guidance", ["कृपया अपने आदेश की पुष्टि करें", self.text_menu])
        self.exec_audio_block_gesture("confirmation")
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "back":
                self.stop_hotword()
                self.current_process = 25
                self.stop_get_menu_voice()
                print("back pressed")
                return
            if self.button_pressed == "confirm":
                self.stop_hotword()
                self.current_process = 27
                self.stop_get_menu_voice()
                print("back pressed")
                return

    # 27
    def bill(self):
        self.change_page("qrCodebillPage", ["कृपया बिल का भुगतान करने के लिए क्यूआर कोड को स्कैन करें", self.text_menu])
        self.animation_change("qrCodePage", ["qr"], "jpg")
        self.exec_audio_block_gesture("bill")
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "back":
                self.stop_hotword()
                self.current_process = 26
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "confirm":
                self.stop_hotword()
                self.current_process = 28
                self.nav_joy()
                self.stop_get_menu_voice()
                print("back pressed")
                return

    # 28
    def kitchen(self):

        self.change_page("kitchenOptions", ["कृपया आदेश की जांच करें।", self.text_menu])
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "deliver":
                self.stop_hotword()
                self.current_process = 29
                self.nav_joy()
                self.stop_get_menu_voice()
                print("back pressed")
                return

            if self.button_pressed == "opentray":
                self.button_pressed = ""
                self.tray_cnt.publish("open")

    # 29
    def moveon(self):
        self.change_page("moveon")
        self.exec_audio_block_gesture("moveon")
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "confirm":
                self.button_pressed = ""
                self.current_process = 30
                self.change_page("enjoyfood")
                self.exec_audio_block_gesture("enjoy")
                self.nav_joy()
                self.stop_get_menu_voice()
                print("back pressed")
                return

    # 30
    def tray_close(self):
        self.change_page("trayclose")
        # self.exec_audio_block_gesture("trayclose")

        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "trayclose":
                self.button_pressed = ""
                self.tray_cnt.publish("close")
                self.nav_joy()
                self.current_process = 0
                return

    # 31
    def train_details(self):
        self.change_page("inputNumber")
        self.exec_audio_block_gesture("train")
        self.button_pressed = ""
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.button_pressed == "back" or self.voice_result_menu == "ok back":
                self.stop_hotword()
                self.current_process = 10
                self.stop_get_menu_voice()
                print("back pressed")
                return

    def nav_joy(self):
        self.nav_ended = ""
        self.change_page("homePage")
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.nav_ended == "done":
                break

    # def feedback_func(self):
    #     self.change_feedback_page()
    #     self.start_get_menu_voice(["next"])
    #     self.button_pressed = ""
    #
    #     # self.exec_audio("")
    #     while not rospy.is_shutdown():
    #         time.sleep(0.5)
    #
    #         if self.button_pressed == "next" or self.voice_result_menu == "ok next":
    #             self.current_process = 10
    #             self.stop_get_menu_voice()
    #             print("back pressed")
    #             return
    #     self.stop_hotword()
    #     self.stop_get_menu_voice()
    # 10

    # 14
    def guidance_flow(self):
        if self.all_tour == True:
            self.guidance_current_zone += 1
            if self.guidance_current_zone > 9:
                self.all_tour = False
                self.guidance_current_zone = 10

        self.robot_goal_position = self.guidance_current_zone
        if self.robot_goal_position == self.robot_current_position:
            self.current_process = 13  # guidance menu
            return

        self.current_process = 18

    # 18

    def guidance_confirmation(self):

        f = self.confirmation_dict[self.guidance_current_zone]
        f()

        if self.zone_no_of_end_position_dict[self.robot_current_position] <= 1:
            if not self.faq_called:
                if self.guidance_current_zone != 0:
                    self.exec_audio_block("say_hi")
                    time.sleep(1)
                self.exec_audio_block_gesture("conf_" + self.zone_no_name_dict[self.guidance_current_zone])

        self.faq_called = False

        if self.zone_no_of_end_position_dict[self.robot_current_position] > 1:
            if not self.zone_second_position_reached:
                name = self.zone_no_name_dict[self.robot_current_position] + "_1"
                self.navigate_with_position(name)
                self.zone_second_position_reached = True

        self.start_get_menu_voice(["ok back", "ok confirm"])
        self.button_pressed = ""
        # self.twitter_selected = False
        self.start_hotword()
        cnt = 0
        while not rospy.is_shutdown():
            time.sleep(0.5)
            cnt += 1
            # if self.button_pressed == "back" or self.voice_result_menu == "ok back":
            if self.button_pressed == "back" or self.voice_result_menu == "ok back":

                self.all_tour = False
                self.stop_get_menu_voice()
                print("back pressed")
                self.stop_hotword()
                if self.guidance_current_zone == 10:
                    self.current_process = 15  # zone
                    self.guidance_current_zone = 0
                else:
                    self.current_process = 13  # guidance menu
                return
            # if self.button_pressed == "confirm" or self.voice_result_menu == "ok confirm":
            if self.button_pressed == "confirm" or self.voice_result_menu == "ok confirm":
                self.current_process = 15  # zone

                self.stop_hotword()
                self.stop_get_menu_voice()
                return

            if self.hotword_result == "wake" and not self.guidance_current_zone == 0:  # FOR 10 SECONDS
                self.current_process = 11
                self.process_after_faq = 18
                self.stop_get_menu_voice()

                return

            if cnt == 50:  # at 25 th seconds
                self.stop_get_menu_voice()

            if cnt > 3600:
                self.all_tour = False
                self.stop_get_menu_voice()

                self.stop_hotword()
                self.current_process = 10
                return
        self.stop_hotword()
        self.stop_get_menu_voice()

    def twitter_guide(self):

        self.navigate_with_position("twitter")

        t2 = threading.Thread(target=self.execute_hand_raises, args=(self.zone_gesture_dict[10],))
        t2.start()
        time.sleep(3)
        self.exec_audio_block("twitter")
        #        self.exec_audio_block("hand_" + self.zone_no_name_dict[10])
        t2.join()

        # self.twitter_selected = False
        time.sleep(2)

    # 15

    def guidance_zones(self):
        # if self.guidance_current_zone == 0 and self.twitter_selected:
        #     self.twitter_guide()

        f = self.zone_dict[self.guidance_current_zone]
        f()

        if self.guidance_current_zone != 0:
            self.exec_audio_block_gesture("hpcl1")
        nav_res = self.navigate()
        if nav_res == "cancelled_inside_dxc":
            self.current_process = 13
            return
        elif nav_res == "cancelled_outside_dxc":
            self.current_process = 1
            return
        self.robot_current_position = self.robot_goal_position
        self.zone_second_position_reached = False
        # to close the guidance while it r eaches home
        if self.guidance_current_zone == 0:
            self.guidance_started = False
            self.current_process = 1
            return
        else:
            self.guidance_started = True

        t2 = threading.Thread(target=self.execute_hand_raises,
                              args=(self.zone_gesture_dict[self.guidance_current_zone],))
        t2.start()
        time.sleep(3)
        self.exec_audio_block("hand_" + self.zone_no_name_dict[self.guidance_current_zone])
        t2.join()
        self.exec_audio_block_gesture("narration_" + self.zone_no_name_dict[self.guidance_current_zone])

        if self.guidance_current_zone == 10:  # after twitter closing
            self.current_process = 15
            self.guidance_current_zone = 0
            time.sleep(4)
            self.exec_audio_block_gesture("going_home")
            return

        if self.all_tour:
            self.current_process = 14

        else:
            self.current_process = 13

        # time.sleep(1)
        # self.long_speech_gesture.publish('start')
        # self.exec_audio_block("hpcl3")
        # self.long_speech_gesture.publish('stop')
        # self.start_hotword()
        # cnt = 0
        # while not rospy.is_shutdown():
        #     time.sleep(0.5)
        #     cnt += 1
        #
        #     if self.hotword_result == "wake":  # FOR 10 SECONDS
        #         if self.all_tour:
        #             self.current_process = 11
        #             self.process_after_faq = 14
        #
        #         else:
        #             self.current_process = 11
        #             self.process_after_faq = 13
        #         self.stop_hotword()
        #         return
        #
        #     if cnt > 20:
        #         if self.all_tour:
        #             self.current_process = 14
        #
        #         else:
        #             self.current_process = 13
        #         self.stop_hotword()
        #         return
        #
        # self.stop_hotword()

    def door_entry(self, name="infrontofdoor"):

        self.nav_result = ""
        self.button_pressed = ""
        self.movebase_publish(self.goals[name]["x"], self.goals[name]["y"], self.goals[name]["z"],
                              self.goals[name]["w"])

        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.nav_result != "":
                # audio needs change
                self.exec_audio_block_gesture("hpcl2")
                self.nav_result = ""
                self.change_page("entrance")
                self.start_get_menu_voice(["ok proceed", "ok return"])

            # if self.button_pressed == "go" or self.voice_result_menu == "ok go":
            if self.button_pressed == "proceed" or self.voice_result_menu == "proceed" or self.voice_result_menu == "ok proceed":
                self.stop_get_menu_voice()
                f = self.zone_dict[self.guidance_current_zone]
                f()
                # audio needs change
                self.exec_audio_block_gesture("thank_door")
                return "done"

            if self.button_pressed == "return" or self.voice_result_menu == "ok return":
                self.stop_get_menu_voice()
                return "back"

    ###
    def feedback_func1(self):
        self.change_page("feedBack")
        self.button_pressed = ""

        # self.exec_audio("askfeedback")

        question = 0
        while not rospy.is_shutdown():

            time.sleep(0.5)
            print("question no", question)

            if question == 0:
                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb1")

                # self.change_feedback_questions("multiple_choice",question)

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs

            if (question == 1 and self.button_pressed == "next" and self.answer) or (
                    question == 1 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue
                self.answers.append(self.answer)
                print(self.answers)
                self.change_feedback_questions("multiple_choice", question)
                self.exec_audio_block("fb2")

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs
                self.answer = []

            if (question == 2 and self.button_pressed == "next" and self.answer) or (
                    question == 2 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue

                self.answers.append(self.answer)
                print(self.answers)

                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb3")

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs
                self.answer = []

            if (question == 3 and self.button_pressed == "next" and self.answer) or (
                    question == 3 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue

                self.answers.append(self.answer)
                print(self.answers)

                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb4")

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs
                self.answer = []

            if (question == 4 and self.button_pressed == "next" and self.answer) or (
                    question == 4 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue

                self.answers.append(self.answer)
                print(self.answers)

                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb5")

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs
                self.answer = []

            if (question == 5 and self.button_pressed == "next" and self.answer) or (
                    question == 5 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue

                self.answers.append(self.answer)
                print(self.answers)

                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb6")

                question = question + 1
                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                start_time = rospy.Time.now().secs
                self.answer = []

            if (question == 6 and self.button_pressed == "next" and self.answer) or (
                    question == 6 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    print("ques6")
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                print("ques6 answer")
                                break
                            if not self.answer:
                                print("ques6 no answer")
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        print("ques6 continue")
                        continue

                self.answers.append(self.answer)
                print(self.answers)

                self.change_feedback_questions("radio", question)
                self.exec_audio_block("fb7")

                self.button_pressed = ""
                self.stop_get_menu_voice()
                self.start_get_menu_voice(["ok next"])
                question = 100
                print("nextpressed")
                start_time = rospy.Time.now().secs
                self.answer = []

            current_time = rospy.Time.now().secs
            print((current_time - start_time))

            if (question == 100 and self.button_pressed == "next" and self.answer) or (
                    current_time - start_time) >= self.feedback_question_timeout or (
                    question == 100 and self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "feedBack", "button": "next"})

                    self.button_control_pub.publish(dict)

                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_question_timeout:
                            self.voicebutton_response = False
                            if self.answer:
                                break
                            if not self.answer:
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice(["ok next"])
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        continue

                if (current_time - start_time) >= self.feedback_question_timeout:
                    self.current_process = 10
                    self.answers = []
                    self.answer = []
                    self.stop_get_menu_voice()
                    return

                else:
                    self.answers.append(self.answer)
                    self.answer = []

                    print(self.answers, "last question")
                    self.stop_get_menu_voice()

                    self.current_process = 16
                    print(self.current_process)
                    return

        # self.stop_hotword()
        # self.stop_get_menu_voice()

    ###
    def feedback_func2(self):
        start_time = rospy.Time.now().secs

        print("func2")

        self.change_page("voiceFeedBack")

        dict = {"textbox1": ""}  # emprty text box
        self.update_texts("voiceFeedBack", dict)

        self.exec_audio_block("fb8")

        self.button_pressed = ""

        # self.exec_audio("say_feedback_voice")
        self.start_get_feedback_voice()
        self.start_get_menu_voice(["ok next"])

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().secs

            time.sleep(0.5)
            if self.voice_feedback_received:
                dict = {"textbox1": self.voice_result_feedback}
                self.update_texts("voiceFeedBack", dict)
                self.voice_feedback_received = False

            if (self.button_pressed == "next" and self.answer) or (
                    current_time - start_time) >= self.feedback_voice_timeout or (
                    self.voice_result_menu == "ok next"):
                if self.voice_result_menu == "ok next":
                    dict = json.dumps({"screen_id": "voiceFeedBack", "button": "next"})

                    self.button_control_pub.publish(dict)
                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_voice_timeout:
                            self.voicebutton_response = False
                            break

                        time.sleep(0.5)

                self.answers.extend(self.answer)
                print(self.answers)
                self.answer = []

                self.current_process = 17
                self.stop_get_feedback_voice()
                self.stop_get_menu_voice()

                print("next pressed")
                return
        # #self.stop_hotword()
        # self.stop_get_feedback_voice()
        # self.stop_get_menu_voice()
        # print("func2end")

    ###
    def feedback_func3(self):
        start_time = rospy.Time.now().secs

        self.change_page("feedBackSubmit")
        dict = {"textbox1": "", "textbox2": "", "textbox3": "", "textbox4": ""}  # empty voice feedback textbox
        self.update_texts("feedBackSubmit", dict)

        self.start_get_menu_voice(["ok submit", "ok cancel"])
        self.start_get_no_voice()
        self.button_pressed = ""
        feteched_data = False

        # while not rospy.is_shutdown() or not self.formdb_control_result :
        #     time.sleep(0.5) #wait for database
        #     print("waiting for database results")
        #     pass
        # self.update_uidata_db(self.formdb_control_result)

        # self.exec_audio("")
        print("phone_umber", self.phonenumber)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().secs

            time.sleep(0.5)

            if self.voice_result_number != "":
                dict = {"textbox1": self.voice_result_number}
                self.update_texts("feedBackSubmit", dict)
                self.voice_result_number = ""

            print("voiceno", self.voice_result_number, len(self.voice_result_number))
            if (len(self.voice_result_number) == 10 and not self.formdb_control_result):
                dict = {"query": ["NAME", "EMPLOYEE_ID", "EMAIL_ID"], "phone_number": self.voice_result_number,
                        "control": "start"}
                self.checkdb_pub.publish(json.dumps(dict))
                self.voice_result_number = ""

            if (self.phonenumber != "" and not self.formdb_control_result):
                dict = {"query": ["NAME", "EMPLOYEE_ID", "EMAIL_ID"], "phone_number": self.phonenumber,
                        "control": "start"}
                self.checkdb_pub.publish(json.dumps(dict))
                self.phonenumber = ""

            print(self.formdb_control_result)
            print(self.phonenumber)

            if self.formdb_control_result:
                if self.formdb_control_result != ["notfound"]:

                    dict = {"textbox2": self.formdb_control_result[0], "textbox3": self.formdb_control_result[1],
                            "textbox4": self.formdb_control_result[2]}  # empty voice feedback textbox
                    self.update_texts("feedBackSubmit", dict)

                    self.formdb_control_result = []

                    feteched_data = True

                elif self.formdb_control_result == ["notfound"]:
                    dict = {"textbox2": "Not found", "textbox3": "Not found",
                            "textbox4": "Not found"}  # not found voice feedback textbox
                    self.update_texts("feedBackSubmit", dict)
                    self.formdb_control_result = []
                    feteched_data = False

            if self.button_pressed == "submit" or (
                    current_time - start_time) >= self.feedback_submit_timeout or self.voice_result_menu == "ok submit":
                print("submit")
                if self.voice_result_menu == "ok submit":
                    dict = json.dumps({"screen_id": "feedBackSubmit", "button": "submit"})

                    self.button_control_pub.publish(dict)
                    print("publised func3 voice")
                    while not rospy.is_shutdown():
                        current_time = rospy.Time.now().secs
                        if self.voicebutton_response or (current_time - start_time) >= self.feedback_submit_timeout:
                            print((current_time - start_time))
                            self.voicebutton_response = False
                            if self.answer:
                                print("voicebutton answer", self.answer)
                                break
                            if not self.answer:
                                print("voicebutton notanswer", self.answer)
                                self.stop_get_menu_voice()
                                self.start_get_menu_voice()
                                break

                        time.sleep(0.5)
                    if not self.answer:
                        print("voicebutton notanswer continue", self.answer)
                        continue

                if feteched_data or (current_time - start_time) >= self.feedback_submit_timeout:
                    self.answers.extend(self.answer)
                    print(self.answers, len(self.answers))

                    dict = json.dumps({"insert_data": self.answers})  # insert answers to database
                    if len(self.answers) == 9:
                        self.insertdb_pub.publish(dict)

                    self.answer = []
                    self.answers = []
                    self.phonenumber = ""
                    self.formdb_control_result = []

                    dict = {"textbox1": "", "textbox2": "", "textbox3": "",
                            "textbox4": ""}  # empty submit feedback textbox
                    self.update_texts("feedBackSubmit", dict)

                    dict = {"textbox1": ""}  # empty voice feedback textbox
                    self.update_texts("voiceFeedBack", dict)

                    self.current_process = 10  # 10

                    # list=self.fetch_userdetails(phonenumber)
                    #
                    self.stop_get_menu_voice()
                    self.stop_get_no_voice()
                    print("submit pressed")
                    return

            if self.button_pressed == "cancel" or (
                    current_time - start_time) >= self.feedback_submit_timeout or self.voice_result_menu == "ok cancel":
                print("cancel")

                self.answer = []
                self.answers = []
                self.phonenumber = ""
                self.formdb_control_result = []

                dict = {"textbox1": "", "textbox2": "", "textbox3": "", "textbox4": ""}  # empty submit feedback textbox
                self.update_texts("feedBackSubmit", dict)
                dict = {"textbox1": ""}  # empty voice feedback textbox
                self.update_texts("voiceFeedBack", dict)

                self.current_process = 10  # 10

                # list=self.fetch_userdetails(phonenumber)
                #
                self.stop_get_menu_voice()
                self.stop_get_no_voice()
                print("submit pressed")
                return

        # self.stop_hotword()
        self.stop_get_menu_voice()
        self.stop_get_no_voice()

    def navigate_with_position(self, name):
        self.nav_result = ""
        self.movebase_publish(self.goals[name]["x"], self.goals[name]["y"], self.goals[name]["z"],
                              self.goals[name]["w"])
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.nav_result != "":
                self.nav_result = ""
                self.navigating = False
                return
        self.navigating = False
        self.cancel_goal.publish("")

    def navigate(self):
        self.navigating = True
        out = "done"

        if self.robot_current_position == 0:
            res = self.door_entry("infrontofdoor")
            self.robot_current_position = 11
            if res == "back":
                # audio needs change
                self.exec_audio_block_gesture("hpcl2")
                self.guidance_current_zone = 0
                out = "cancelled_outside_dxc"

        if self.robot_goal_position in [9, 0] and self.robot_current_position in [1, 2, 3, 4, 5, 6, 7, 8, 12]:
            res = self.door_entry("doorreturn")
            self.robot_current_position = 12
            if res == "back":
                out = "cancelled_inside_dxc"
                return out

        self.robot_goal_position = self.guidance_current_zone
        name = self.zone_no_name_dict[self.guidance_current_zone]
        self.nav_result = ""
        self.movebase_publish(self.goals[name]["x"], self.goals[name]["y"], self.goals[name]["z"],
                              self.goals[name]["w"])
        while not rospy.is_shutdown():
            time.sleep(0.5)
            if self.nav_result != "":
                self.nav_result = ""
                self.navigating = False
                self.robot_current_position = self.robot_goal_position
                return out
        self.navigating = False
        self.cancel_goal.publish("")
        return out

    ##############

    def change_page(self, screen_id, list=[]):
        dict1 = {"screen_id": screen_id,
                 "contents": {"texts": list, "button_labels": [],
                              "button_enable": {}, "video": "false"}}
        self.ui_control.publish(json.dumps(dict1))

    def change_page_zone(self, screen_id, zone_id, list=[]):
        dict1 = {"screen_id": screen_id, "zone_id": zone_id,
                 "contents": {"texts": list, "button_labels": [], "button_enabe": {},
                              "video": "false"}}
        self.ui_control.publish(json.dumps(dict1))

    ##

    def update_texts(self, screen_id, dict={}):
        dict2 = {"screen_id": screen_id, "texts": dict}
        self.ui_textsupdate.publish(json.dumps(dict2))

    def update_texts_zone(self, screen_id, zone_id, dict={}):
        dict2 = {"screen_id": screen_id, "texts": dict}
        self.ui_textsupdate.publish(json.dumps(dict2))

    ##

    def animation_change(self, screen_id, list=[], format=""):
        dict3 = {"screen_id": screen_id, "name": list, "format": format}
        self.animation_pub.publish(json.dumps(dict3))

    def animation_change_zone(self, screen_id, zone_id, list=[], format=""):
        dict3 = {"screen_id": screen_id, "zone_id": zone_id, "name": list, "format": format}
        self.animation_pub.publish(json.dumps(dict3))

    ##
    def change_feedback_questions(self, feedback_type, question):
        dict2 = {"screen_id": "feedBack", "type": feedback_type, "question": self.questions[question],
                 "choices": self.choices[question]}
        self.ui_textsupdate.publish(json.dumps(dict2))

    ############

    # def change_mask_temp_status(self, maskstatus, temp="Measuring in"):
    #     tempvalue = temp + " "
    #     dict2 = {"screen_id": "covidScreening", "texts": {"temperature": tempvalue, "mask_status": maskstatus.upper()}}
    #     self.update.publish(json.dumps(dict2))

    # def update_ph_no_ui(self, phone_number):
    #     self.ph_no = ""
    #     dict1 = {"screen_id": "inputNumber",
    #              "texts": {"phone_number": phone_number}}
    #     self.ui_textsupdate.publish(json.dumps(dict1))

    # def update_screen_ph_no(self):

    #     dict2 = {"screen_id": "inputNumber", "texts": {"phone_number": self.voice_result_number}}
    #     self.update.publish(json.dumps(dict2))

    # def update_screen_feedbackvoice(self, texts):
    #     # dict1={ "screen_id":"voiceFeedBack", "contents":{"texts":["g"],"button_labels":[], "button_enabe":{},"video":"false"}}
    #     # dict2={ "screen_id":"voiceFeedBack", "texts":["ghhjhjhjj  vhvh"]}

    #     # self.ui_control.publish(json.dumps(dict1))
    #     # self.ui_textsupdate.publish(json.dumps(dict2))
    #     dict2 = {"screen_id": "voiceFeedBack", "texts": texts}
    #     self.ui_textsupdate.publish(json.dumps(dict2))

    # def update_ph_no_feedback(self, phone_number):
    #     dict1 = {"screen_id": "feedBackSubmit",
    #              "texts": {"phone_number": phone_number}}
    #     self.ui_textsupdate.publish(json.dumps(dict1))

    # def update_screen_feedbacksubmit(self, textbox_content):

    #     dict2 = {"screen_id": "feedBackSubmit", "texts": textbox_content}
    #     self.ui_textsupdate.publish(json.dumps(dict2))

    # def update_uidata_db(self, userdetails_list):
    #     dict2 = {"screen_id": "feedBackSubmit", "texts": userdetails_list}
    #     self.ui_textsupdate.publish(json.dumps(dict2))

    ##Zones

    def change_confirmation_z0(self):

        self.change_page_zone("subMenu", "shZone", ["Closing Guidance"])

    def change_confirmation_z1(self):

        self.change_page_zone("subMenu", "tlZone", ["TimeLine Zone"])

        #  self.animation_change_zone("subMenu","shZone",["Timeline"],"png")

    def change_confirmation_z2(self):

        self.change_page_zone("subMenu", "shZone", ["Smart Helmet and AR Zone"])

        self.animation_change_zone("subMenu", "shZone", ["Smart helmet"], "png")
        #

    # def change_confirmation_z2(self):
    #     dict1 = {"screen_id": "subMenu", "zone_id": "arZone",
    #              "contents": {"texts": ["Augumented Reality Zone"], "button_labels": [], "button_enabe": {},
    #                           "video": "false"}}
    #     dict3 = {"screen_id": "subMenu", "zone_id": "arZone", "name": ["AR Zone"], "format": "png"}
    #     self.ui_control.publish(json.dumps(dict1))
    #     self.animation_pub.publish(json.dumps(dict3))

    def change_confirmation_z3(self):

        self.change_page_zone("subMenu", "ceZone", ["Central Exploratory Zone"])

        self.animation_change_zone("subMenu", "ceZone", ["Central zone"], "png")

    def change_confirmation_z4(self):
        self.change_page_zone("subMenu", "vmZone", ["VR and MR Zone"])
        self.animation_change_zone("subMenu", "vmZone", ["AR & MR Zone"], "png")

    def change_confirmation_z5(self):

        self.change_page_zone("subMenu", "slZone", ["Self Learning Zone"])
        self.animation_change_zone("subMenu", "slZone", ["Self learning Zone"], "png")

    def change_confirmation_z6(self):

        self.change_page_zone("subMenu", "mpZone", ["Multitouch Phygital zone"])
        self.animation_change_zone("subMenu", "mpZone", ["Multi touch zone"], "png")

    def change_confirmation_z7(self):

        self.change_page_zone("subMenu", "ccZone", ["Command and Control Centre"])
        self.animation_change_zone("subMenu", "ccZone", ["Command zone"], "png")

    def change_confirmation_z8(self):
        self.change_page_zone("subMenu", "hZone", ["Holographic Zone"])
        self.animation_change_zone("subMenu", "hZone", ["Holographic zone"], "png")

    def change_confirmation_z9(self):

        self.change_page_zone("subMenu", "shZone", ["Feedback Zone"])
        self.animation_change_zone("subMenu", "shZone", ["Holographic zone"], "png")

    def change_confirmation_z10(self):

        self.change_page_zone("subMenu", "shZone", ["Twitter Wall"])
        self.animation_change_zone("subMenu", "shZone", ["Holographic zone"], "png")

    def change_training(self):

        self.change_page("robotWalkAnimation", ["Face Training on Progress ... "])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_picture1(self, name):

        self.change_page("robotWalkAnimation", ["Hi I am SAYA "])
        self.animation_change("robotWalkAnimation", [name], "mp4")

    def change_picture2(self):

        self.change_page("robotWalkAnimation", ["Hi I am SAYA "])
        self.animation_change("robotWalkAnimation", ["asimov"], "mp4")

    def change_z0(self):
        self.change_page("robotWalkAnimation", ["Going to home"])

    def change_z1(self):

        self.change_page("robotWalkAnimation", ["TimeLine Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z2(self):

        self.change_page("robotWalkAnimation", ["Smart Helmet Zone and AR Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

        # def change_z2(self):

    #     dict1 = {"screen_id": "robotWalkAnimation",
    #              "contents": {"texts": ["AR Zone"], "button_labels": [], "button_enabe": {}, "video": "false"}}
    #     dict3 = {"screen_id": "robotWalkAnimation", "name": ["screensaver"], "format": "mp4"}
    #     self.ui_control.publish(json.dumps(dict1))
    #     self.animation_pub.publish(json.dumps(dict3))

    def change_z3(self):
        self.change_page("robotWalkAnimation", ["Central Exploratory Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z4(self):
        self.change_page("robotWalkAnimation", ["VR and MR Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z5(self):
        self.change_page("robotWalkAnimation", ["Self Learning Zone"])

        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z6(self):
        self.change_page("robotWalkAnimation", ["multitouch phygital Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z7(self):
        self.change_page("robotWalkAnimation", ["Command and Control Centre Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z8(self):
        self.change_page("robotWalkAnimation", ["holographic Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z9(self):
        self.change_page("robotWalkAnimation", ["Feedback Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_z10(self):
        self.change_page("robotWalkAnimation", ["Twitter Wall"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def change_charge(self):
        self.change_page("robotWalkAnimation", ["Feedback Zone"])
        self.animation_change("robotWalkAnimation", ["screensaver"], "mp4")

    def main_process(self):
        self.current_process = 0

        while not rospy.is_shutdown():
            time.sleep(1)
            # self.stop_faq()
            self.stop_hotword()
            self.stop_get_menu_voice()

            print(self.current_process)

            if self.current_process == 0:
                print(self.current_process)
                self.home_page()
            elif self.current_process == 1:
                print(self.current_process)

                self.wait_for_person()
            elif self.current_process == 2:
                print(self.current_process)

                self.check_face()
            elif self.current_process == 3:
                print(self.current_process)

                self.check_mask()
            elif self.current_process == 4:
                print(self.current_process)

                self.check_temp_sanitize()
            elif self.current_process == 5:
                print(self.current_process)

                self.registration_deciding1()
            elif self.current_process == 6:
                print(self.current_process)

                self.phone_no_entry()
            elif self.current_process == 7:
                print(self.current_process)

                self.capture_image()
            elif self.current_process == 8:
                print(self.current_process)

                self.display_qr()
            elif self.current_process == 9:
                print(self.current_process)

                self.guidance_entry()
            elif self.current_process == 10:
                print(self.current_process)

                self.options_showing()
            elif self.current_process == 11:
                print(self.current_process)

                self.faq()
            elif self.current_process == 13:
                print(self.current_process)

                self.guidance_menu()
            elif self.current_process == 12:
                print(self.current_process)

                self.feedback_func1()
            elif self.current_process == 14:
                print(self.current_process)

                self.guidance_flow()
            elif self.current_process == 15:
                print(self.current_process)

                self.guidance_zones()
            elif self.current_process == 16:
                print(self.current_process)

                self.feedback_func2()
            elif self.current_process == 17:
                print(self.current_process)

                self.feedback_func3()
            elif self.current_process == 18:
                print(self.current_process)
                self.guidance_confirmation()

            elif self.current_process == 19:
                print(self.current_process)
                self.battery_charging()

            elif self.current_process == 22:
                print(self.current_process)
                self.settings_page()
            elif self.current_process == 23:
                print(self.current_process)
                self.training()
            elif self.current_process == 24:
                print(self.current_process)
                self.menu_page()
            elif self.current_process == 25:
                print(self.current_process)
                self.quick_menu()
            elif self.current_process == 26:
                print(self.current_process)
                self.confirm()
            elif self.current_process == 27:
                print(self.current_process)
                self.bill()
            elif self.current_process == 28:
                print(self.current_process)
                self.kitchen()
            elif self.current_process == 29:
                print(self.current_process)
                self.moveon()
            elif self.current_process == 30:
                print(self.current_process)
                self.tray_close()
            elif self.current_process == 31:
                print(self.current_process)
                self.train_details()
            elif self.current_process == 32:
                print(self.current_process)
                self.detail_menu()


if __name__ == '__main__':
    rospy.init_node("main_flow", anonymous=True)
    test = flow()
    test.main_process()






