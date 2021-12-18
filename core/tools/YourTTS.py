"""
MIT License

Copyright (c) 2020 rootadminWalker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""
import os

import librosa
import torch.cuda
from TTS.tts.utils.speakers import SpeakerManager
from playsound import playsound

from .Abstract import Tools

try:
    from TTS.utils.audio import AudioProcessor
except ImportError:
    from TTS.utils.audio import AudioProcessor

from TTS.tts.models import setup_model
from TTS.config import load_config
from TTS.tts.models.vits import *

"""
YourTTS implementation instance, download the model and the configs from:
https://drive.google.com/drive/folders/12_Vmsbp7v5oVaqIRcfnWe1BLcJHLzkZz?usp=sharing

Clone TTS package with: git clone https://github.com/Edresson/Coqui-TTS -b multilingual-torchaudio-SE TTS
and run pip inside it

For some example reference audios, you can view here:
https://drive.google.com/drive/folders/161ZQmwZbgByXQt4PEbRjSLhtOZe2oUXs?usp=sharing
"""


class YourTTS(Tools):
    BEST_MODEL = 'best_model.pth.tar'
    CONFIG = 'config.json'
    CONFIG_SE = 'config_se.json'
    LANGUAGE_IDS = 'language_ids.json'
    SE_CHECKPOINT = 'SE_checkpoint.pth.tar'
    SPEAKERS = 'speakers.json'

    def __init__(self, model_directory_path, reference_files, use_cuda):
        """
        This instance implements the YourTTS imitation, to do text-to-speech without using robotic vocals
        You should have a directory with the models stored inside:
            - best_model.pth.tar
            - config.json
            - config_se.json
            - language_ids.json
            - SE_checkpoint.pth.tar
            - speaker.json
        Args:
            model_directory_path: The path to the directory with yourtts models stored.
            reference_files: The reference audios in wav for voice imitation. If it's a directory, the instance will
                            parse every wav file inside it; If it's a file, then it will just parse it
            use_cuda: Use cuda if you want (Not recommended)
        """

        # load the config and the base model
        self.C, self.model = self.__load_base_model(model_directory_path, use_cuda)

        # load the audio processor
        self.audio_processor = AudioProcessor(**self.C.audio)

        SE_speaker_manager = self.__load_speaker_encoder(model_directory_path, use_cuda)
        if os.path.isdir(reference_files):
            self.__reference_files = [os.path.join(reference_files, wav) for wav in os.listdir(reference_files)]
        elif reference_files.endswith('.wav'):
            self.__reference_files = [reference_files]
        else:
            raise ValueError(f"Your path {reference_files} is neither a directory or a wav audio file")

        self.reference_emb = SE_speaker_manager.compute_d_vector_from_clip(self.__reference_files)

    @staticmethod
    def __load_base_model(model_directory_path, use_cuda):
        model_path = os.path.join(model_directory_path, YourTTS.BEST_MODEL)
        config_path = os.path.join(model_directory_path, YourTTS.CONFIG)
        tts_languages = os.path.join(model_directory_path, YourTTS.LANGUAGE_IDS)
        tts_speakers = os.path.join(model_directory_path, YourTTS.SPEAKERS)

        C = load_config(config_path)

        C.model_args['d_vector_file'] = tts_speakers
        C.model_args['use_speaker_encoder_as_loss'] = False

        model = setup_model(C)
        model.language_manager.set_language_ids_from_file(tts_languages)

        cp = torch.load(model_path, map_location=torch.device('cpu'))

        # remove speaker encoder
        model_weights = cp['model'].copy()
        for key in list(model_weights.keys()):
            if "speaker_encoder" in key:
                del model_weights[key]

        model.load_state_dict(model_weights)
        model.eval()

        if use_cuda:
            model = model.cuda()

        return C, model

    @staticmethod
    def __load_speaker_encoder(model_directory_path, use_cuda):
        checkpoint_se_path = os.path.join(model_directory_path, YourTTS.SE_CHECKPOINT)
        config_se_path = os.path.join(model_directory_path, YourTTS.CONFIG_SE)

        # synthesize voice
        SE_speaker_manager = SpeakerManager(encoder_model_path=checkpoint_se_path, encoder_config_path=config_se_path,
                                            use_cuda=use_cuda)
        return SE_speaker_manager

    def say(self, text, length_scale=1.1, inference_noise_scale=0.3, inference_noise_scale_dp=0.3, language_id=0):
        """
        You can call this method to speak it out, remember don't delete or modify the file /tmp/voice.wav
        Args:
            text: The text you want the speaker to speak
            length_scale: Default is 1.1. scaler for the duration predictor. The larger it is, the slower the speech.
            inference_noise_scale: Default is 0.3, defines the noise variance applied to the random z vector at inference.
            inference_noise_scale_dp: Default is 0.3. defines the noise variance applied to the duration predictor z vector at inference.
            language_id: Default is 0, english. You won't use others, will you? (If so, check is in language_ids.json)

        Returns:

        """
        self.model.length_scale = length_scale
        self.model.inference_noise_scale = inference_noise_scale
        self.model.inference_noise_scale_dp = inference_noise_scale_dp

        wav, alignment, _, _ = synthesis(
            self.model,
            text,
            self.C,
            "cuda" in str(next(self.model.parameters()).device),
            self.audio_processor,
            speaker_id=None,
            d_vector=self.reference_emb,
            style_wav=None,
            language_id=language_id,
            enable_eos_bos_chars=self.C.enable_eos_bos_chars,
            use_griffin_lim=True,
            do_trim_silence=False
        ).values()
        self.audio_processor.save_wav(wav, '/tmp/voice.wav')

        playsound('/tmp/voice.wav')

    def __compute_spec(self, ref_file):
        y, sr = librosa.load(ref_file, sr=self.audio_processor.sample_rate)
        spec = self.audio_processor.spectrogram(y)
        spec = torch.FloatTensor(spec).unsqueeze(0)
        return spec
