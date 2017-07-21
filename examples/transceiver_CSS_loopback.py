#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: IEEE 802.15.4 Transceiver using CSS PHY
# Description: IEEE 802.15.4 Transceiver using CSS PHY
# Generated: Wed Jun 15 20:35:27 2016
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

from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
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
import wx


class transceiver_CSS_loopback(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="IEEE 802.15.4 Transceiver using CSS PHY")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.text_msg = text_msg = "This is a test message"
        self.c = c = ieee802_15_4.css_phy(chirp_number=4, phy_packetsize_bytes=len(text_msg)+15)
        self.samp_rate = samp_rate = 32e6
        self.phi = phi = 10
        self.pad_len = pad_len = c.nsamp_frame*0
        self.msg_interval = msg_interval = 1000
        self.freq_off = freq_off = 0
        self.enable = enable = 1.0
        self.delay = delay = 0
        self.ampl = ampl = 0

        ##################################################
        # Blocks
        ##################################################
        _pad_len_sizer = wx.BoxSizer(wx.VERTICAL)
        self._pad_len_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_pad_len_sizer,
        	value=self.pad_len,
        	callback=self.set_pad_len,
        	label="Pad Length",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._pad_len_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_pad_len_sizer,
        	value=self.pad_len,
        	callback=self.set_pad_len,
        	minimum=0,
        	maximum=10000,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_pad_len_sizer)
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
        	label="Message Interval [ms]",
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
        _freq_off_sizer = wx.BoxSizer(wx.VERTICAL)
        self._freq_off_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_freq_off_sizer,
        	value=self.freq_off,
        	callback=self.set_freq_off,
        	label="Frequency Offset",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._freq_off_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_freq_off_sizer,
        	value=self.freq_off,
        	callback=self.set_freq_off,
        	minimum=-1e5,
        	maximum=1e5,
        	num_steps=1000,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_freq_off_sizer)
        _delay_sizer = wx.BoxSizer(wx.VERTICAL)
        self._delay_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_delay_sizer,
        	value=self.delay,
        	callback=self.set_delay,
        	label="Delay",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._delay_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_delay_sizer,
        	value=self.delay,
        	callback=self.set_delay,
        	minimum=0,
        	maximum=100000,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_delay_sizer)
        _ampl_sizer = wx.BoxSizer(wx.VERTICAL)
        self._ampl_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_ampl_sizer,
        	value=self.ampl,
        	callback=self.set_ampl,
        	label="Noise Amplitude",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._ampl_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_ampl_sizer,
        	value=self.ampl,
        	callback=self.set_ampl,
        	minimum=0,
        	maximum=1,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_ampl_sizer)
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
        	baseband_freq=0,
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
        _phi_sizer = wx.BoxSizer(wx.VERTICAL)
        self._phi_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_phi_sizer,
        	value=self.phi,
        	callback=self.set_phi,
        	label="Phi",
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._phi_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_phi_sizer,
        	value=self.phi,
        	callback=self.set_phi,
        	minimum=2,
        	maximum=20,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_phi_sizer)
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
        self._enable_chooser = forms.button(
        	parent=self.GetWin(),
        	value=self.enable,
        	callback=self.set_enable,
        	label="TX Enable",
        	choices=[1.0, 0.0],
        	labels=['on', 'off'],
        )
        self.Add(self._enable_chooser)
        self.blocks_vector_insert_x_0 = blocks.vector_insert_c(([0 for i in range(pad_len)]), 3*c.nsamp_frame, 3*c.nsamp_frame)
        self.blocks_socket_pdu_0_0 = blocks.socket_pdu("UDP_SERVER", "", "52001", 10000, False)
        self.blocks_multiply_xx_1 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.intern(text_msg), msg_interval)
        self.blocks_delay_0 = blocks.delay(gr.sizeof_gr_complex*1, int(delay))
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_SIN_WAVE, freq_off, 1, 0)
        self.analog_fastnoise_source_x_0 = analog.fastnoise_source_c(analog.GR_GAUSSIAN, ampl, 0, 8192)
        self.analog_const_source_x_0 = analog.sig_source_c(0, analog.GR_CONST_WAVE, 0, 0, 10)

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
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_delay_0, 0))    
        self.connect((self.analog_fastnoise_source_x_0, 0), (self.blocks_add_xx_0, 0))    
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 0))    
        self.connect((self.blocks_add_xx_0, 0), (self.ieee802_15_4_css_phy_1, 0))    
        self.connect((self.blocks_add_xx_0, 0), (self.wxgui_fftsink2_0, 0))    
        self.connect((self.blocks_add_xx_0, 0), (self.wxgui_scopesink2_2, 0))    
        self.connect((self.blocks_add_xx_0, 0), (self.wxgui_waterfallsink2_0, 0))    
        self.connect((self.blocks_delay_0, 0), (self.blocks_multiply_xx_1, 0))    
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_vector_insert_x_0, 0))    
        self.connect((self.blocks_multiply_xx_1, 0), (self.blocks_add_xx_0, 1))    
        self.connect((self.blocks_vector_insert_x_0, 0), (self.blocks_multiply_xx_1, 1))    
        self.connect((self.ieee802_15_4_css_phy_1, 0), (self.blocks_multiply_xx_0, 1))    
        self.connect((self.ieee802_15_4_css_phy_1, 1), (self.wxgui_scopesink2_3, 0))    

    def get_text_msg(self):
        return self.text_msg

    def set_text_msg(self, text_msg):
        self.text_msg = text_msg
        self.set_c(ieee802_15_4.css_phy(chirp_number=4, phy_packetsize_bytes=len(self.text_msg)+15))
        self.blocks_message_strobe_0.set_msg(pmt.intern(self.text_msg))

    def get_c(self):
        return self.c

    def set_c(self, c):
        self.c = c
        self.set_pad_len(self.c.nsamp_frame*0)
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

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)
        self.wxgui_scopesink2_2.set_sample_rate(self.samp_rate)
        self.wxgui_scopesink2_3.set_sample_rate(self.samp_rate)
        self.wxgui_waterfallsink2_0.set_sample_rate(self.samp_rate)

    def get_phi(self):
        return self.phi

    def set_phi(self, phi):
        self.phi = phi
        self._phi_slider.set_value(self.phi)
        self._phi_text_box.set_value(self.phi)

    def get_pad_len(self):
        return self.pad_len

    def set_pad_len(self, pad_len):
        self.pad_len = pad_len
        self._pad_len_slider.set_value(self.pad_len)
        self._pad_len_text_box.set_value(self.pad_len)

    def get_msg_interval(self):
        return self.msg_interval

    def set_msg_interval(self, msg_interval):
        self.msg_interval = msg_interval
        self._msg_interval_slider.set_value(self.msg_interval)
        self._msg_interval_text_box.set_value(self.msg_interval)
        self.blocks_message_strobe_0.set_period(self.msg_interval)

    def get_freq_off(self):
        return self.freq_off

    def set_freq_off(self, freq_off):
        self.freq_off = freq_off
        self._freq_off_slider.set_value(self.freq_off)
        self._freq_off_text_box.set_value(self.freq_off)
        self.analog_sig_source_x_0.set_frequency(self.freq_off)

    def get_enable(self):
        return self.enable

    def set_enable(self, enable):
        self.enable = enable
        self._enable_chooser.set_value(self.enable)

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self._delay_slider.set_value(self.delay)
        self._delay_text_box.set_value(self.delay)
        self.blocks_delay_0.set_dly(int(self.delay))

    def get_ampl(self):
        return self.ampl

    def set_ampl(self, ampl):
        self.ampl = ampl
        self._ampl_slider.set_value(self.ampl)
        self._ampl_text_box.set_value(self.ampl)
        self.analog_fastnoise_source_x_0.set_amplitude(self.ampl)


def main(top_block_cls=transceiver_CSS_loopback, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
