# licenta
#!/usr/bin/env python3
# -- coding: utf-8 --

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Masurare BW canal
# Author: YB
# GNU Radio version: 3.10.5.1

from packaging.version import Version as StrictVersion

if _name_ == '_main_':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import eng_notation
from gnuradio import qtgui
import sip
from gnuradio import analog
from gnuradio import blocks
from gnuradio import fft
from gnuradio.fft import window
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import iio
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore
import Masurare_BW_canal_baleiaj_frecventa as baleiaj_frecventa  # embedded python module
import Masurare_BW_canal_epy_block_0 as epy_block_0  # embedded python block
import Masurare_BW_canal_epy_module_0 as epy_module_0  # embedded python module
import time
import threading



from gnuradio import qtgui

class Masurare_BW_canal(gr.top_block, Qt.QWidget):

    def _init_(self):
        gr.top_block._init_(self, "Masurare BW canal", catch_exceptions=True)
        Qt.QWidget._init_(self)
        self.setWindowTitle("Masurare BW canal")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "Masurare_BW_canal")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.nivel = nivel = 0
        self.samp_rate = samp_rate = 50000000
        self.prag = prag = (-80)
        self.freq = freq = baleiaj_frecventa.sweeper(nivel)/1e6
        self.out = out = epy_module_0.detect(nivel,samp_rate,prag,freq)
        self.nr_puncte_fft = nr_puncte_fft = 1024
        self.freq_min = freq_min = freq*1e6-samp_rate/2
        self.freq_canal = freq_canal = out[0]
        self.bw = bw = out[1]
        self.pozitie = pozitie = int((float(freq_canal)-freq_min)/(samp_rate)*nr_puncte_fft)
        self.latime_banda = latime_banda = bw
        self.frecventa = frecventa = freq
        self.castig = castig = 50
        self.canal = canal = freq_canal

        ##################################################
        # Blocks
        ##################################################

        self._prag_range = Range((-100), (-70), 1, (-80), 200)
        self._prag_win = RangeWidget(self._prag_range, self.set_prag, "Prag", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._prag_win)
        self._castig_range = Range(10, 100, 1, 50, 200)
        self._castig_win = RangeWidget(self._castig_range, self.set_castig, "Câștig ARF", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._castig_win)
        self.qtgui_vector_sink_f_0 = qtgui.vector_sink_f(
            int(nr_puncte_fft),
            (freq*1e6-samp_rate/2),
            (samp_rate/nr_puncte_fft),
            "Frecventa [MHz]",
            "Nivel [dBm]",
            "",
            3, # Number of inputs
            None # parent
        )
        self.qtgui_vector_sink_f_0.set_update_time(0.10)
        self.qtgui_vector_sink_f_0.set_y_axis((-100), (-40))
        self.qtgui_vector_sink_f_0.enable_autoscale(False)
        self.qtgui_vector_sink_f_0.enable_grid(False)
        self.qtgui_vector_sink_f_0.set_x_axis_units("MHz")
        self.qtgui_vector_sink_f_0.set_y_axis_units("dBm")
        self.qtgui_vector_sink_f_0.set_ref_level(0)


        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(3):
            if len(labels[i]) == 0:
                self.qtgui_vector_sink_f_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_vector_sink_f_0.set_line_label(i, labels[i])
            self.qtgui_vector_sink_f_0.set_line_width(i, widths[i])
            self.qtgui_vector_sink_f_0.set_line_color(i, colors[i])
            self.qtgui_vector_sink_f_0.set_line_alpha(i, alphas[i])

        self._qtgui_vector_sink_f_0_win = sip.wrapinstance(self.qtgui_vector_sink_f_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_vector_sink_f_0_win)
        self.prob_nivel = blocks.probe_signal_vf(nr_puncte_fft)
        def _nivel_probe():
          while True:

            val = self.prob_nivel.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_nivel,val))
              except AttributeError:
                self.set_nivel(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (2))
        _nivel_thread = threading.Thread(target=_nivel_probe)
        _nivel_thread.daemon = True
        _nivel_thread.start()
        self._latime_banda_tool_bar = Qt.QToolBar(self)

        if None:
            self._latime_banda_formatter = None
        else:
            self._latime_banda_formatter = lambda x: eng_notation.num_to_str(x)

        self._latime_banda_tool_bar.addWidget(Qt.QLabel("Latime de banda [kHz]"))
        self._latime_banda_label = Qt.QLabel(str(self._latime_banda_formatter(self.latime_banda)))
        self._latime_banda_tool_bar.addWidget(self._latime_banda_label)
        self.top_layout.addWidget(self._latime_banda_tool_bar)
        self.iio_pluto_source_0 = iio.fmcomms2_source_fc32('' if '' else iio.get_pluto_uri(), [True, True], 32768)
        self.iio_pluto_source_0.set_len_tag_key('packet_len')
        self.iio_pluto_source_0.set_frequency((int(freq*1e6)))
        self.iio_pluto_source_0.set_samplerate(samp_rate)
        self.iio_pluto_source_0.set_gain_mode(0, 'manual')
        self.iio_pluto_source_0.set_gain(0, castig)
        self.iio_pluto_source_0.set_quadrature(True)
        self.iio_pluto_source_0.set_rfdc(True)
        self.iio_pluto_source_0.set_bbdc(True)
        self.iio_pluto_source_0.set_filter_params('Auto', '', 0, 0)
        self._frecventa_tool_bar = Qt.QToolBar(self)

        if None:
            self._frecventa_formatter = None
        else:
            self._frecventa_formatter = lambda x: eng_notation.num_to_str(x)

        self._frecventa_tool_bar.addWidget(Qt.QLabel("Frecventa:  "))
        self._frecventa_label = Qt.QLabel(str(self._frecventa_formatter(self.frecventa)))
        self._frecventa_tool_bar.addWidget(self._frecventa_label)
        self.top_layout.addWidget(self._frecventa_tool_bar)
        self.fft_vxx_0 = fft.fft_vcc(int(nr_puncte_fft), True, window.blackmanharris(nr_puncte_fft), True, 1)
        self.epy_block_0 = epy_block_0.blk(pos_init=pozitie, pos_end=pozitie+bw, veclen=nr_puncte_fft)
        self._canal_tool_bar = Qt.QToolBar(self)

        if None:
            self._canal_formatter = None
        else:
            self._canal_formatter = lambda x: eng_notation.num_to_str(x)

        self._canal_tool_bar.addWidget(Qt.QLabel("Canal radio gasit [MHz]"))
        self._canal_label = Qt.QLabel(str(self._canal_formatter(self.canal)))
        self._canal_tool_bar.addWidget(self._canal_label)
        self.top_layout.addWidget(self._canal_tool_bar)
        self.blocks_stream_to_vector_1 = blocks.stream_to_vector(gr.sizeof_float*1, nr_puncte_fft)
        self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, int(nr_puncte_fft))
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(10, int(nr_puncte_fft), (-100))
        self.blocks_integrate_xx_0 = blocks.integrate_ff(100, int(nr_puncte_fft))
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(int(nr_puncte_fft))
        self.analog_const_source_x_0 = analog.sig_source_f(0, analog.GR_CONST_WAVE, 0, 0, prag)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_stream_to_vector_1, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.blocks_integrate_xx_0, 0))
        self.connect((self.blocks_integrate_xx_0, 0), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.prob_nivel, 0))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.qtgui_vector_sink_f_0, 1))
        self.connect((self.blocks_stream_to_vector_0, 0), (self.fft_vxx_0, 0))
        self.connect((self.blocks_stream_to_vector_1, 0), (self.qtgui_vector_sink_f_0, 0))
        self.connect((self.epy_block_0, 0), (self.qtgui_vector_sink_f_0, 2))
        self.connect((self.fft_vxx_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.iio_pluto_source_0, 0), (self.blocks_stream_to_vector_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "Masurare_BW_canal")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_nivel(self):
        return self.nivel

    def set_nivel(self, nivel):
        self.nivel = nivel
        self.set_freq(baleiaj_frecventa.sweeper(self.nivel)/1e6)
        self.set_out(epy_module_0.detect(self.nivel,self.samp_rate,self.prag,self.freq))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_freq_min(self.freq*1e6-self.samp_rate/2)
        self.set_out(epy_module_0.detect(self.nivel,self.samp_rate,self.prag,self.freq))
        self.set_pozitie(int((float(self.freq_canal)-self.freq_min)/(self.samp_rate)*self.nr_puncte_fft))
        self.iio_pluto_source_0.set_samplerate(self.samp_rate)
        self.qtgui_vector_sink_f_0.set_x_axis((self.freq*1e6-self.samp_rate/2), (self.samp_rate/self.nr_puncte_fft))

    def get_prag(self):
        return self.prag

    def set_prag(self, prag):
        self.prag = prag
        self.set_out(epy_module_0.detect(self.nivel,self.samp_rate,self.prag,self.freq))
        self.analog_const_source_x_0.set_offset(self.prag)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_frecventa(self.freq)
        self.set_freq_min(self.freq*1e6-self.samp_rate/2)
        self.set_out(epy_module_0.detect(self.nivel,self.samp_rate,self.prag,self.freq))
        self.iio_pluto_source_0.set_frequency((int(self.freq*1e6)))
        self.qtgui_vector_sink_f_0.set_x_axis((self.freq*1e6-self.samp_rate/2), (self.samp_rate/self.nr_puncte_fft))

    def get_out(self):
        return self.out

    def set_out(self, out):
        self.out = out
        self.set_bw(self.out[1])
        self.set_freq_canal(self.out[0])

    def get_nr_puncte_fft(self):
        return self.nr_puncte_fft

    def set_nr_puncte_fft(self, nr_puncte_fft):
        self.nr_puncte_fft = nr_puncte_fft

        self.set_pozitie(int((float(self.freq_canal)-self.freq_min)/(self.samp_rate)*self.nr_puncte_fft))

        self.epy_block_0.veclen = self.nr_puncte_fft
        self.qtgui_vector_sink_f_0.set_x_axis((self.freq*1e6-self.samp_rate/2), (self.samp_rate/self.nr_puncte_fft))

    def get_freq_min(self):
        return self.freq_min

    def set_freq_min(self, freq_min):
        self.freq_min = freq_min
        self.set_pozitie(int((float(self.freq_canal)-self.freq_min)/(self.samp_rate)*self.nr_puncte_fft))

    def get_freq_canal(self):
        return self.freq_canal

    def set_freq_canal(self, freq_canal):
        self.freq_canal = freq_canal
        self.set_canal(self.freq_canal)
        self.set_pozitie(int((float(self.freq_canal)-self.freq_min)/(self.samp_rate)*self.nr_puncte_fft))

    def get_bw(self):
        return self.bw

    def set_bw(self, bw):
        self.bw = bw
        self.set_latime_banda(self.bw)
        self.epy_block_0.pos_end = self.pozitie+self.bw

    def get_pozitie(self):
        return self.pozitie

    def set_pozitie(self, pozitie):
        self.pozitie = pozitie
        self.epy_block_0.pos_end = self.pozitie+self.bw
        self.epy_block_0.pos_init = self.pozitie

    def get_latime_banda(self):
        return self.latime_banda

    def set_latime_banda(self, latime_banda):
        self.latime_banda = latime_banda
        Qt.QMetaObject.invokeMethod(self._latime_banda_label, "setText", Qt.Q_ARG("QString", str(self._latime_banda_formatter(self.latime_banda))))

    def get_frecventa(self):
        return self.frecventa

    def set_frecventa(self, frecventa):
        self.frecventa = frecventa
        Qt.QMetaObject.invokeMethod(self._frecventa_label, "setText", Qt.Q_ARG("QString", str(self._frecventa_formatter(self.frecventa))))

    def get_castig(self):
        return self.castig

    def set_castig(self, castig):
        self.castig = castig
        self.iio_pluto_source_0.set_gain(0, self.castig)

    def get_canal(self):
        return self.canal

    def set_canal(self, canal):
        self.canal = canal
        Qt.QMetaObject.invokeMethod(self._canal_label, "setText", Qt.Q_ARG("QString", str(self._canal_formatter(self.canal))))




def main(top_block_cls=Masurare_BW_canal, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    print('before exec')
    qapp.exec_()
    print('after exec')

if _name_ == '_main_':
    main()
