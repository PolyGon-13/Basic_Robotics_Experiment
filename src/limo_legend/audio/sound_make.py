from gtts import gTTS
import os

text = "게임을 시작하지"
tts = gTTS(text=text, lang='ko')
tts.save("start.mp3")
print("end")
