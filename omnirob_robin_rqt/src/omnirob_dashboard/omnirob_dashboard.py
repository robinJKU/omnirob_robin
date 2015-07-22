from rqt_gui_py.plugin import Plugin
from .omnirob_dashboard_widget import OmnirobDashboardWidget

class OmnirobDashboard(Plugin):

    def __init__(self, context):
        super(OmnirobDashboard, self).__init__(context)
        self.setObjectName('OmnirobDashboard')

        self._widget = OmnirobDashboardWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    #def save_settings(self, plugin_settings, instance_settings):
    #    self._widget.save_settings(plugin_settings, instance_settings)

    #def restore_settings(self, plugin_settings, instance_settings):
    #    self._widget.restore_settings(plugin_settings, instance_settings)

    #def trigger_configuration(self):
    #    self._widget.trigger_configuration()
