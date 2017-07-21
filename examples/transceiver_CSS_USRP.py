#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: IEEE 802.15.4 Transceiver using CSS PHY
# Description: IEEE 802.15.4 Transceiver using CSS PHY
# Generated: Wed Jun 22 12:23:47 2016
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

import os
import sys
sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from gnuradio.wxgui import forms
from gnuradio.wxgui import scopesink2
from gnuradio.wxgui import waterfallsink2
from grc_gnuradio import wxgui as grc_wxgui
from ieee802_15_4_css_phy import ieee802_15_4_css_phy  # grc-generated hier_block
from optparse import OptionParser
import ieee802_15_4
import pmt
import time
import trafficGenerator
import wx


class transceiver_CSS_USRP(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="IEEE 802.15.4 Transceiver using CSS PHY")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.text_msg = text_msg = "Hello World, this is GNU Radio using the IEEE 802.15.4 CSS PHY!"
        self.freq = freq = 2480000000
        self.samp_rate = samp_rate = 1e6
        self.msg_interval = msg_interval = 1000
        self.gain = gain = 20
        self.cur_freq = cur_freq = freq
        self.c = c = ieee802_15_4.css_phy(chirp_number=4, phy_packetsize_bytes=len(text_msg)+15)

        ##################################################
        # Blocks
        ##################################################
        self.nb = self.nb = wx.Notebook(self.GetWin(), style=wx.NB_TOP)
        self.nb.AddPage(grc_wxgui.Panel(self.nb), "RX Waterfall")
        self.nb.AddPage(grc_wxgui.Panel(self.nb), "RX FFT")
        self.nb.AddPage(grc_wxgui.Panel(self.nb), "RX Time")
        self.nb.AddPage(grc_wxgui.Panel(self.nb), "RX Symbols")
        self.Add(self.nb)
        _msg_interval_sizer = wx.BoxSizer(wx.VERTICAL)
        self._msg_interval_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_msg_interval_sizer,
        	value=self.msg_interval,
        	callback=self.set_msg_interval,
        	label="Message interval [ms]",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._msg_interval_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_msg_interval_sizer,
        	value=self.msg_interval,
        	callback=self.set_msg_interval,
        	minimum=1,
        	maximum=5000,
        	num_steps=1000,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_msg_interval_sizer)
        _gain_sizer = wx.BoxSizer(wx.VERTICAL)
        self._gain_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_gain_sizer,
        	value=self.gain,
        	callback=self.set_gain,
        	label="TX/RX Gain",
        	converter=forms.int_converter(),
        	proportion=0,
        )
        self._gain_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_gain_sizer,
        	value=self.gain,
        	callback=self.set_gain,
        	minimum=1,
        	maximum=100,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=int,
        	proportion=1,
        )
        self.Add(_gain_sizer)
        self._freq_chooser = forms.radio_buttons(
        	parent=self.GetWin(),
        	value=self.freq,
        	callback=self.set_freq,
        	label="Channel",
        	choices=[1000000 * (2400 + 5 * (i - 10)) for i in range(11, 27)],
        	labels=[i for i in range(11, 27)],
        	style=wx.RA_HORIZONTAL,
        )
        self.Add(self._freq_chooser)
        self.wxgui_waterfallsink2_0 = waterfallsink2.waterfall_sink_c(
        	self.nb.GetPage(0).GetWin(),
        	baseband_freq=0,
        	dynamic_range=100,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=512,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="RX Waterfall",
        )
        self.nb.GetPage(0).Add(self.wxgui_waterfallsink2_0.win)
        self.wxgui_scopesink2_3 = scopesink2.scope_sink_c(
        	self.nb.GetPage(3).GetWin(),
        	title="RX Correlator Output",
        	sample_rate=samp_rate,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=1,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label="Counts",
        )
        self.nb.GetPage(3).Add(self.wxgui_scopesink2_3.win)
        self.wxgui_scopesink2_2 = scopesink2.scope_sink_c(
        	self.nb.GetPage(2).GetWin(),
        	title="RX Time Signal",
        	sample_rate=samp_rate,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=1,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label="Counts",
        )
        self.nb.GetPage(2).Add(self.wxgui_scopesink2_2.win)
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.nb.GetPage(1).GetWin(),
        	baseband_freq=freq,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=True,
        	avg_alpha=None,
        	title="RX FFT",
        	peak_hold=False,
        )
        self.nb.GetPage(1).Add(self.wxgui_fftsink2_0.win)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("addr=192.168.10.4", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(gain, 0)
        self.uhd_usrp_source_0.set_antenna("J1", 0)
        self.uhd_usrp_sink_1 = uhd.usrp_sink(
        	",".join(("addr=192.168.10.6", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_1.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_1.set_center_freq(freq, 0)
        self.uhd_usrp_sink_1.set_gain(gain, 0)
        self.trafficGenerator_Distribution_0 = trafficGenerator.Distribution(0, 100, 4, 5, int)
        self.ieee802_15_4_rime_stack_0 = ieee802_15_4.rime_stack(([129]), ([131]), ([132]), ([23,42]))
        self.ieee802_15_4_mac_0 = ieee802_15_4.mac(True)
        self.ieee802_15_4_css_phy_1 = ieee802_15_4_css_phy(
            bits_per_cw=c.bits_per_symbol,
            chirp_seq=c.chirp_seq,
            codewords=c.codewords,
            intlv_seq=c.intlv_seq,
            len_sub=38,
            nbytes_payload=c.phy_packetsize_bytes,
            nsamp_frame=c.nsamp_frame,
            num_subchirps=c.n_subchirps,
            nzeros_padding=c.padded_zeros,
            phr=c.PHR,
            preamble=c.preamble,
            sfd=c.SFD,
            sym_per_frame=c.nsym_frame,
            threshold=0.95,
            time_gap_1=c.time_gap_1,
            time_gap_2=c.time_gap_2,
        )
        self._cur_freq_static_text = forms.static_text(
        	parent=self.GetWin(),
        	value=self.cur_freq,
        	callback=self.set_cur_freq,
        	label="Current center frequency",
        	converter=forms.float_converter(),
        )
        self.Add(self._cur_freq_static_text)
        self.blocks_socket_pdu_0_0 = blocks.socket_pdu("UDP_SERVER", "", "52001", 10000, False)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_int*1)
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.intern(text_msg), msg_interval)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_message_strobe_0, 'strobe'), (self.ieee802_15_4_rime_stack_0, 'bcin'))    
        self.msg_connect((self.blocks_socket_pdu_0_0, 'pdus'), (self.ieee802_15_4_rime_stack_0, 'bcin'))    
        self.msg_connect((self.ieee802_15_4_css_phy_1, 'rxout'), (self.ieee802_15_4_mac_0, 'pdu in'))    
        self.msg_connect((self.ieee802_15_4_mac_0, 'pdu out'), (self.ieee802_15_4_css_phy_1, 'txin'))    
        self.msg_connect((self.ieee802_15_4_mac_0, 'app out'), (self.ieee802_15_4_rime_stack_0, 'fromMAC'))    
        self.msg_connect((self.ieee802_15_4_rime_stack_0, 'bcout'), (self.blocks_socket_pdu_0_0, 'pdus'))    
        self.msg_connect((self.ieee802_15_4_rime_stack_0, 'toMAC'), (self.ieee802_15_4_mac_0, 'app in'))    
        self.connect((self.ieee802_15_4_css_phy_1, 0), (self.uhd_usrp_sink_1, 0))    
        self.connect((self.ieee802_15_4_css_phy_1, 1), (self.wxgui_scopesink2_3, 0))    
        self.connect((self.trafficGenerator_Distribution_0, 0), (self.blocks_null_sink_0, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.ieee802_15_4_css_phy_1, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.wxgui_fftsink2_0, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.wxgui_scopesink2_2, 0))    
        self.connect((self.uhd_usrp_source_0, 0), (self.wxgui_waterfallsink2_0, 0))    

    def get_text_msg(self):
        return self.text_msg

    def set_text_msg(self, text_msg):
        self.text_msg = text_msg
        self.set_c(ieee802_15_4.css_phy(chirp_number=4, phy_packetsize_bytes=len(self.text_msg)+15))
        self.blocks_message_strobe_0.set_msg(pmt.intern(self.text_msg))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_cur_freq(self.freq)
        self._freq_chooser.set_value(self.freq)
        self.uhd_usrp_sink_1.set_center_freq(self.freq, 0)
        self.uhd_usrp_source_0.set_center_freq(self.freq, 0)
        self.wxgui_fftsink2_0.set_baseband_freq(self.freq)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_1.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)
        self.wxgui_scopesink2_2.set_sample_rate(self.samp_rate)
        self.wxgui_scopesink2_3.set_sample_rate(self.samp_rate)
        self.wxgui_waterfallsink2_0.set_sample_rate(self.samp_rate)

    def get_msg_interval(self):
        return self.msg_interval

    def set_msg_interval(self, msg_interval):
        self.msg_interval = msg_interval
        self._msg_interval_slider.set_value(self.msg_interval)
        self._msg_interval_text_box.set_value(self.msg_interval)
        self.blocks_message_strobe_0.set_period(self.msg_interval)

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain
        self._gain_slider.set_value(self.gain)
        self._gain_text_box.set_value(self.gain)
        self.uhd_usrp_sink_1.set_gain(self.gain, 0)
        	
        self.uhd_usrp_source_0.set_gain(self.gain, 0)
        	

    def get_cur_freq(self):
        return self.cur_freq

    def set_cur_freq(self, cur_freq):
        self.cur_freq = cur_freq
        self._cur_freq_static_text.set_value(self.cur_freq)

    def get_c(self):
        return self.c

    def set_c(self, c):
        self.c = c
        self.ieee802_15_4_css_phy_1.set_bits_per_cw(self.c.bits_per_symbol)
        self.ieee802_15_4_css_phy_1.set_chirp_seq(self.c.chirp_seq)
        self.ieee802_15_4_css_phy_1.set_codewords(self.c.codewords)
        self.ieee802_15_4_css_phy_1.set_intlv_seq(self.c.intlv_seq)
        self.ieee802_15_4_css_phy_1.set_nbytes_payload(self.c.phy_packetsize_bytes)
        self.ieee802_15_4_css_phy_1.set_nsamp_frame(self.c.nsamp_frame)
        self.ieee802_15_4_css_phy_1.set_num_subchirps(self.c.n_subchirps)
        self.ieee802_15_4_css_phy_1.set_nzeros_padding(self.c.padded_zeros)
        self.ieee802_15_4_css_phy_1.set_phr(self.c.PHR)
        self.ieee802_15_4_css_phy_1.set_preamble(self.c.preamble)
        self.ieee802_15_4_css_phy_1.set_sfd(self.c.SFD)
        self.ieee802_15_4_css_phy_1.set_sym_per_frame(self.c.nsym_frame)
        self.ieee802_15_4_css_phy_1.set_time_gap_1(self.c.time_gap_1)
        self.ieee802_15_4_css_phy_1.set_time_gap_2(self.c.time_gap_2)


def main(top_block_cls=transceiver_CSS_USRP, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
