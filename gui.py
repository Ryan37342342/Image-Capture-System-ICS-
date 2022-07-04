import rospy
from kivy.lang import Builder
from kivymd.app import MDApp
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.screen import MDScreen


class IcsGuiApp(MDApp):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)

        self.screen=Builder.load_file('/home/aaeon/ICS/src/ICS/gui.kv')

    def build(self):
        return self.screen

if __name__ == '__main__':
    rospy.init_node("gui_node", anonymous=True)

    IcsGuiApp().run()
