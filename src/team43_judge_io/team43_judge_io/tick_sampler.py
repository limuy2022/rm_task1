"""Measurement-stamped snapshot sampling for the nine-column tick contract."""
from dataclasses import dataclass
import math
import numpy as np
from team43_common.time_buffer import TimeBuffer


@dataclass
class TickSnapshot:
    prediction: np.ndarray
    current: np.ndarray
    base: np.ndarray
    advice: np.ndarray
    target_valid: bool
    base_valid: bool
    advice_valid: bool


class TickSampler:
    def __init__(self, state_timeout=.2, observation_timeout=.5):
        self.state_timeout=float(state_timeout)
        self.observation_timeout=float(observation_timeout)
        if (not all(math.isfinite(v) and v > 0 for v in (self.state_timeout, self.observation_timeout))):
            raise ValueError('timeouts must be finite and positive')
        self.target=TimeBuffer();self.base=TimeBuffer();self.advice=TimeBuffer()

    def clear(self):
        self.target.clear();self.base.clear();self.advice.clear()

    def put_target(self,stamp,position,velocity,acceleration,*,valid=True,observation_age=0.):
        state=np.concatenate((np.asarray(position,float).reshape(2),
             np.asarray(velocity,float).reshape(2),np.asarray(acceleration,float).reshape(2)))
        ok=bool(valid) and np.all(np.isfinite(state)) and math.isfinite(float(observation_age)) and observation_age>=0
        self.target.put(stamp,(state,bool(ok),float(observation_age)))

    def put_base(self,stamp,position,velocity,*,valid=True):
        state=np.concatenate((np.asarray(position,float).reshape(2),np.asarray(velocity,float).reshape(2)))
        self.base.put(stamp,(state,bool(valid) and bool(np.all(np.isfinite(state)))))

    def put_advice(self,stamp,velocity,*,valid=True):
        value=np.asarray(velocity,float).reshape(2)
        self.advice.put(stamp,(value,bool(valid) and bool(np.all(np.isfinite(value)))))

    def sample(self,stamp,prediction_horizon=.1):
        if not math.isfinite(float(prediction_horizon)) or prediction_horizon < 0:
            raise ValueError('prediction horizon must be finite and nonnegative')
        t=float(stamp);cur=np.zeros(2);pred=cur.copy();base=cur.copy();vel=cur.copy()
        tv=bv=av=False
        item=self.target.before(t,self.state_timeout)
        if item is not None:
            source,(x,ok,age)=item;dt=max(0.,t-source)
            tv=ok and age+dt <= self.observation_timeout
            if tv:
                cur=x[:2]+x[2:4]*dt+.5*x[4:6]*dt*dt
                h=dt+float(prediction_horizon)
                pred=x[:2]+x[2:4]*h+.5*x[4:6]*h*h
        item=self.base.before(t,self.state_timeout)
        if item is not None:
            source,(x,bv)=item
            if bv:base=x[:2]+x[2:4]*max(0.,t-source)
        item=self.advice.before(t,self.state_timeout)
        if item is not None:
            source,(value,av)=item
            if av:vel=value.copy()
        return TickSnapshot(pred,cur,base,vel,tv,bool(bv),bool(av))
