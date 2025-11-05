import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class CraneFIS:
    def __init__(self):
        self.u_bl = np.arange(0, 121, 1) # Boom length 0..120 m
        self.u_cgd = np.arange(0, 41, 0.1) # CG distance 0..40 m
        self.u_ph = np.arange(-30, 61, 0.5) # Payload height -30..60m

        self.u_ctrl = np.arange(0, 3.01, 0.01) # ControlAdjustment 0..3
        self.u_feed = np.arange(0, 3.01, 0.01) # OperatorFeedback 0..3 

        # Antecedents / Consequents
        self.BoomLength = ctrl.Antecedent(self.u_bl, 'BoomLength')
        self.CGDistance = ctrl.Antecedent(self.u_cgd, 'CGDistance')
        self.PayloadHeight = ctrl.Antecedent(self.u_ph, 'PayloadHeight')

        self.ControlAdjustment = ctrl.Consequent(self.u_ctrl, 'ControlAdjustment')
        self.OperatorFeedback = ctrl.Consequent(self.u_feed, 'OperatorFeedback')

        self._build_mfs()
        self._build_rules()
        self.sim = ctrl.ControlSystemSimulation(self.ctrlsys, flush_after_run=30)

    def _build_mfs(self):
        self.BoomLength['Short']  = fuzz.trapmf(self.BoomLength.universe, [0, 0, 10, 25])
        self.BoomLength['Medium'] = fuzz.trapmf(self.BoomLength.universe, [15, 30, 50, 70])
        self.BoomLength['Long']   = fuzz.trapmf(self.BoomLength.universe, [60, 80, 120, 120])

        self.CGDistance['Close']  = fuzz.trapmf(self.CGDistance.universe, [0, 0, 2, 6])
        self.CGDistance['Medium'] = fuzz.trapmf(self.CGDistance.universe, [4, 8, 12, 18])
        self.CGDistance['Far']    = fuzz.trapmf(self.CGDistance.universe, [14, 20, 40, 40])

        self.PayloadHeight['Low']    = fuzz.trapmf(self.PayloadHeight.universe, [-30, -30, -5, 0])
        self.PayloadHeight['Medium'] = fuzz.trapmf(self.PayloadHeight.universe, [-2, 2, 8, 15])
        self.PayloadHeight['High']   = fuzz.trapmf(self.PayloadHeight.universe, [10, 20, 60, 60])

        self.ControlAdjustment['NoCorrection']    = fuzz.trimf(self.ControlAdjustment.universe, [0.0, 0.0, 1.0])
        self.ControlAdjustment['SmallCorrection'] = fuzz.trimf(self.ControlAdjustment.universe, [0.5, 1.0, 1.5])
        self.ControlAdjustment['StrongCorrection']= fuzz.trimf(self.ControlAdjustment.universe, [1.2, 2.0, 2.5])
        self.ControlAdjustment['OverrideStop']    = fuzz.trimf(self.ControlAdjustment.universe, [2.0, 3.0, 3.0])

        self.OperatorFeedback['Safe']    = fuzz.trimf(self.OperatorFeedback.universe, [0.0, 0.0, 1.0])
        self.OperatorFeedback['Caution'] = fuzz.trimf(self.OperatorFeedback.universe, [0.5, 1.0, 1.5])
        self.OperatorFeedback['Unsafe']  = fuzz.trimf(self.OperatorFeedback.universe, [1.2, 2.0, 2.5])
        self.OperatorFeedback['Danger']  = fuzz.trimf(self.OperatorFeedback.universe, [2.0, 3.0, 3.0])

    def _build_rules(self):
        rules = []
        # simple, overlapping rules to produce continuous outputs across envelope
        rules.append(ctrl.Rule(self.BoomLength['Long'] & self.CGDistance['Far'], 
                               (self.ControlAdjustment['OverrideStop'], self.OperatorFeedback['Danger'])))
        rules.append(ctrl.Rule(self.CGDistance['Close'], 
                               (self.ControlAdjustment['NoCorrection'], self.OperatorFeedback['Safe'])))

        rules.append(ctrl.Rule(self.BoomLength['Medium'] & self.CGDistance['Medium'] & self.PayloadHeight['Medium'],
                               (self.ControlAdjustment['SmallCorrection'], self.OperatorFeedback['Caution'])))

        # Far CG rules (progressively more severe with PH)
        rules.append(ctrl.Rule(self.CGDistance['Far'] & self.PayloadHeight['Low'],
                               (self.ControlAdjustment['SmallCorrection'], self.OperatorFeedback['Caution'])))
        rules.append(ctrl.Rule(self.CGDistance['Far'] & self.PayloadHeight['Medium'],
                               (self.ControlAdjustment['StrongCorrection'], self.OperatorFeedback['Unsafe'])))
        rules.append(ctrl.Rule(self.CGDistance['Far'] & self.PayloadHeight['High'],
                               (self.ControlAdjustment['StrongCorrection'], self.OperatorFeedback['Danger'])))

        # Medium CG with longer boom
        rules.append(ctrl.Rule(self.BoomLength['Long'] & self.CGDistance['Medium'] & self.PayloadHeight['High'],
                               (self.ControlAdjustment['StrongCorrection'], self.OperatorFeedback['Unsafe'])))

        self.ctrlsys = ctrl.ControlSystem(rules)

    def evaluate(self, bl, cgd, ph):
        bl_v = float(np.clip(bl, self.u_bl[0], self.u_bl[-1]))
        cgd_v = float(np.clip(cgd, self.u_cgd[0], self.u_cgd[-1]))
        ph_v = float(np.clip(ph, self.u_ph[0], self.u_ph[-1]))

        try:
            self.sim.input['BoomLength'] = bl_v
            self.sim.input['CGDistance'] = cgd_v
            self.sim.input['PayloadHeight'] = ph_v
            self.sim.compute()
            ca = float(self.sim.output.get('ControlAdjustment', 0.0))
            ofb = float(self.sim.output.get('OperatorFeedback', 0.0))

            if np.isnan(ca) or np.isnan(ofb):
                return 0.0, 0.0
            return ca, ofb
        
        except Exception:
            return 0.0, 0.0
