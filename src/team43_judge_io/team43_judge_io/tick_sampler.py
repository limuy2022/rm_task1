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
    target_fallback: bool = False


class TickSampler:
    def __init__(self, state_timeout=.2, observation_timeout=.5, target_fallback_timeout=.5):
        self.state_timeout=float(state_timeout)
        self.observation_timeout=float(observation_timeout)
        self.target_fallback_timeout=float(target_fallback_timeout)
        if (not all(math.isfinite(v) and v > 0 for v in (self.state_timeout, self.observation_timeout))):
            raise ValueError('timeouts must be finite and positive')
        if not math.isfinite(self.target_fallback_timeout) or not 0 <= self.target_fallback_timeout <= .5:
            raise ValueError('target fallback timeout must be within 0..0.5 seconds')
        self.target=TimeBuffer();self.base=TimeBuffer();self.advice=TimeBuffer()
        self._fallback=TimeBuffer()
        self._target_not_before=0.
        self._last_sample_stamp=None

    def clear_target(self, not_before=0.):
        if float(not_before) < self._target_not_before:
            return # 后到的旧切换事件不能撤销新目标的时间屏障。
        self.target.clear();self._fallback.clear()
        self._target_not_before=float(not_before)

    def clear(self, not_before=0.):
        self._target_not_before=float(not_before)
        self.clear_target(not_before);self.base.clear();self.advice.clear()
        self._last_sample_stamp=None

    def put_target(self,stamp,position,velocity,acceleration,*,valid=True,observation_age=0.):
        stamp,age=float(stamp),float(observation_age)
        if stamp < self._target_not_before or stamp-age < self._target_not_before:
            return
        state=np.concatenate((np.asarray(position,float).reshape(2),
             np.asarray(velocity,float).reshape(2),np.asarray(acceleration,float).reshape(2)))
        ok=bool(valid) and np.all(np.isfinite(state)) and math.isfinite(float(observation_age)) and observation_age>=0
        self.target.put(stamp,(state,bool(ok),float(observation_age)))
        # 只缓存真实有效来源；无效消息、重复采样和CSV补偿都不能续期。
        if (valid and np.all(np.isfinite(state[:2])) and math.isfinite(age)
                and 0 <= age <= self.observation_timeout):
            self._fallback.put(stamp,(state.copy(),age,bool(ok)))

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
        if self._last_sample_stamp is not None and t < self._last_sample_stamp:
            self.clear(not_before=t)
        self._last_sample_stamp=t
        tv=bv=av=False
        fallback=False
        item=self.target.before(t,self.state_timeout)
        if item is not None:
            source,(x,ok,age)=item;dt=max(0.,t-source)
            tv=ok and age+dt <= self.observation_timeout
            if tv:
                cur=x[:2]+x[2:4]*dt+.5*x[4:6]*dt*dt
                h=dt+float(prediction_horizon)
                pred=x[:2]+x[2:4]*h+.5*x[4:6]*h*h
        if not tv and self.target_fallback_timeout > 0:
            item=self._fallback.before(t,self.target_fallback_timeout)
            if item is not None:
                source,(x,age,predictable)=item;dt=max(0.,t-source)
                # 同时限制距最后有效状态和真实图像的年龄，发布外推状态也不能无限续期。
                if age+dt <= self.observation_timeout+self.target_fallback_timeout:
                    cur=x[:2].copy();pred=cur.copy()
                    if predictable:
                        h=dt+float(prediction_horizon)
                        with np.errstate(over='ignore',invalid='ignore'):
                            estimate=x[:2]+x[2:4]*dt+.5*x[4:6]*dt*dt
                            forecast=x[:2]+x[2:4]*h+.5*x[4:6]*h*h
                        if np.all(np.isfinite(estimate)) and np.all(np.isfinite(forecast)):
                            cur,pred=estimate,forecast
                    fallback=True
        item=self.base.before(t,self.state_timeout)
        if item is not None:
            source,(x,bv)=item
            if bv:base=x[:2]+x[2:4]*max(0.,t-source)
        item=self.advice.before(t,self.state_timeout)
        if item is not None:
            source,(value,av)=item
            if av:vel=value.copy()
        return TickSnapshot(pred,cur,base,vel,tv,bool(bv),bool(av),fallback)
