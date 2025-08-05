from gtts import gTTS
from pydub import AudioSegment, silence
import os

phrases = [

    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9","10",
    "11", "12", "13", "14", "15", "16", "17", "18", "19",
    "20", "30", "40", "50", "60", "70", "80", "90","100","1000","thousand",
    "point", "on",
    "satellites", "fix type", "getting fix", "survey", "operating", "RTCM",
    "launch", "outside", "inside", "Base A", "Base B", "course", "time",
    "starting", "error", "information", "landed", "unknown",
    "seconds","minutes", "hours", "radio","gps","baseunit", "gu.unit","cd.unit"
]

outdir = "tts_wavs"
os.makedirs(outdir, exist_ok=True)

for text in phrases:
    tts = gTTS(text=text, lang='en', tld='com')
    mp3_fp = os.path.join(outdir, f"{text.replace(' ', '_')}.mp3")
    wav_fp = os.path.join(outdir, f"{text.replace(' ', '_')}.wav")
    tts.save(mp3_fp)

    audio = AudioSegment.from_mp3(mp3_fp)
    audio = audio.set_frame_rate(8000).set_sample_width(2).set_channels(1)

    # Trim silence from beginning and end
    nonsilent = silence.detect_nonsilent(audio, min_silence_len=100, silence_thresh=-40)
    if nonsilent:
        start = nonsilent[0][0]
        end = nonsilent[-1][1]
        trimmed = audio[start:end]
    else:
        trimmed = audio  # fallback if detection fails

    trimmed.export(wav_fp, format="wav")
    print(f"Generated: {wav_fp}")

    # Delete temporary mp3 file
    try:
        os.remove(mp3_fp)
    except Exception as e:
        print(f"Could not delete {mp3_fp}: {e}")

print("✅ All WAV-files done, trimmed, in 'tts_wavs/'. 8kHz, mono, 16-bit PCM.")

