/* ============================================================================
 * MotionCue-R 비대칭 검출 엔진 (asym_engine.js)
 * ----------------------------------------------------------------------------
 * - 순수 함수/클래스만 포함: DOM·BLE·시계(performance/Date)에 접근하지 않음(시각은 인자 t 로 주입).
 * - 앱(motioncue_app.html), 브라우저 셀프테스트(tests/asym_selftest.html), 리뷰 번들이 **같은 파일**을 사용.
 * - classic script 로 로드(type=module 금지: file:// 에서 CORS 실패). 전역 window.MC 로 노출.
 * - 버전 A6: RawEngine(|a| 러닝 판정·스텝 케이던스·수직축 각속도, yaw_rate=raw 투영/폴백 오일러) + 지표 케이던스 acfPeriod
 *          + 선회/램프/정지 게이트 + Hann 고조파비(진단) + 로브 기반 좌우 짝 SI(불감대·히스테리시스·역할·부호별 절사·0레벨 보정)
 *          + 스트라이드 교차검증(acc·metric·lobe 중앙값) + 상태(hold/not_running/info/ok) + R 기반 스냅샷
 *          + 사전 측정 고정 baseline(BaselineStats/validate/deviation/stale) + 판정기 Decider(2창 연속·만료·세션 불일치) + BaselineStore.
 *          지표: 'yaw_rate'(기본) / 'roll'(사람기준 lean; 'lean' 별칭). SI 는 평균 제거(진동) 기준: 한쪽으로 치우친 자세는 meanOffset 으로 별도.
 * ==========================================================================*/
(function(root){
'use strict';
const ENGINE_VER='A7';

// ------------------------------------------------------------ 상수(GATE) ----
// 모든 게이트 임계는 여기 한 곳. 값의 의미는 algorithm_review/gates_guide.html 참조. 실측 로그로 조정.
const GATE = {
  fsGrid:50,            // 리샘플 격자(Hz) = 기기 알림률(기기 루프는 이보다 빠를 수 없음)
  winSec:15,            // 기본 분석 창(s)
  minSpanFrac:0.98,     // 창 시간범위 충족 비율(미만 → insufficient_duration)
  maxGapMs:250,         // 도착 결손 허용(초과 → sample_gap)
  staleMs:250,          // 마지막 샘플이 이보다 오래되면 stale_data
  maxAbsDeg:60, maxAbsDps:1000,        // 물리적 타당 범위(초과 샘플은 결손으로 제거)
  refJumpDeg:30, refJumpMinSamples:10, // 각도 레벨 시프트(영점 변경 등) → reference_jump
  cadMin:0.7, cadMax:4.0, peakTol:0.03, minStrength:0.5,   // 지표 ACF 주기성 하한(실측 전 잠정 완화 0.6→0.5)
  minAngleStd:1.0, minRateStd:5,
  lobeK:0.15, lobeFloorDeg:0.5, lobeFloorDps:2, lobeExitFrac:0.5, minLobeFrac:0.08, minLobeSamples:3,
  flatRatio:1.15, minLobeRel:0.3, lobeCountLo:0.5, lobeCountHi:1.5,
  pairMin:0.2, pairMax:0.8, minPairs:6, siCapPct:60, trimK:1, trimMinPairs:8, tDisagree:0.30,
  turnMeanDps:15, turnSegDps:25, turnSegSec:2, rampRatio:2.0, eMinFundFrac:0.35, stationaryMaxFrac:0.2,
  // 가속도 기반 러닝 판정/스텝 케이던스 (|a| 노름, 마운트 무관). 실측 전 잠정: 스텝 1.5–5.0 Hz(90–300 spm, 넓게 — 걷기는 std 로 구분),
  // 블록 진입 std 0.20 g / 유지 0.15 g, 주기성 0.35, 창의 60% 블록. 실착용 CSV/창 JSON 으로 재조정 예정
  acc:{ stepMin:1.5, stepMax:5.0, enterStdG:0.20, exitStdG:0.15, minStrength:0.35, blockSec:3, hopSec:1, blockFrac:0.6, stationaryStdG:0.05, enterSec:2, exitSec:3,
        subHarm:0.5 },   // |a| ACF: 선택 랙 절반에 높이 ≥0.5×인 피크가 있으면 스텝으로(스텝/스트라이드 2배 모호 해소)
  standSec:3,
  baseline:{ minWindows:8, floorPct:5, kSigma:2, consecutive:2, offsetWindows:3, staleDays:49, binSpm:15 }
};
// GATE 위에 오버라이드를 얹은 설정 사본(acc/baseline 은 한 단계 깊게 병합). 엔진 생성·튜닝 재분석에 사용
function gateWith(over){ const g=Object.assign({}, GATE, over||{}); g.acc=Object.assign({}, GATE.acc, (over&&over.acc)||{}); g.baseline=Object.assign({}, GATE.baseline, (over&&over.baseline)||{}); return g; }
const REASONS = ['insufficient_duration','stale_data','sample_gap','sample_invalid','reference_jump','turn_contamination',
  'amplitude_ramp','stationary','data_gaps','period_ambiguous','lobe_count','not_running','low_amplitude','weak_periodicity',
  'unclear_alternation','no_raw_data','baseline_mismatch'];

// ---------------------------------------------------------------- 기하 ----
function vperp(u,a){ const d=u[0]*a[0]+u[1]*a[1]+u[2]*a[2]; return [u[0]-d*a[0],u[1]-d*a[1],u[2]-d*a[2]]; }
// a축 기준 u0→u 부호있는 회전각(도): 좌우기울기만 잡고 pitch/yaw엔 불변
function leanAbout(u0,u,a){
  const p0=vperp(u0,a), p=vperp(u,a);
  const s=(p0[1]*p[2]-p0[2]*p[1])*a[0]+(p0[2]*p[0]-p0[0]*p[2])*a[1]+(p0[0]*p[1]-p0[1]*p[0])*a[2];
  const c=p0[0]*p[0]+p0[1]*p[1]+p0[2]*p[2];
  return Math.atan2(s,c)*180/Math.PI;
}
// 쿼터니언(w,x,y,z; 펌웨어 규약 센서→월드) → 중력-up 벡터(센서좌표, 단위벡터) = imu_fusion.c gpred(): Rᵀ·ẑ
function gravityUp(w,x,y,z){
  let up=[2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
  const n=Math.hypot(up[0],up[1],up[2])||1; return [up[0]/n,up[1]/n,up[2]/n];
}
// 수직축 각속도(dps): 센서프레임 자이로(bias 차감)를 중력-up 축에 투영. 양수 = 위에서 볼 때 반시계(왼쪽 회전)
function yawRate(g, up, bias){ const b=bias||[0,0,0]; return (g[0]-b[0])*up[0]+(g[1]-b[1])*up[1]+(g[2]-b[2])*up[2]; }
function angleDeg(u,v){ const d=Math.max(-1,Math.min(1,u[0]*v[0]+u[1]*v[1]+u[2]*v[2])); return Math.acos(d)*180/Math.PI; }

// ---------------------------------------------------------------- DSP ----
function mean(a){ let s=0; for(const v of a) s+=v; return a.length? s/a.length : 0; }
function stdOf(a){ const m=mean(a); let s=0; for(const v of a) s+=(v-m)*(v-m); return a.length? Math.sqrt(s/a.length) : 0; }
function median(a){ if(!a.length) return null; const s=a.slice().sort((x,y)=>x-y), m=s.length>>1; return s.length%2? s[m] : (s[m-1]+s[m])/2; }
function detrend(a){ const n=a.length; let sx=0,sy=0,sxx=0,sxy=0;
  for(let i=0;i<n;i++){ sx+=i; sy+=a[i]; sxx+=i*i; sxy+=i*a[i]; }
  const d=n*sxx-sx*sx||1, m=(n*sxy-sx*sy)/d, b=(sy-m*sx)/n;
  for(let i=0;i<n;i++) a[i]-=(m*i+b); }
// 자기상관 주기(상관계수 정규화·국소피크·"최대−tol 이내 가장 짧은 주기"·포물선 보간). 반환 f0(Hz)|null, strength(0~1), lag
// sub(선택): 선택된 랙 L 의 절반(±20%)에 높이 ≥ sub·r(L) 인 피크가 있으면 그것을 택함 — |a| 처럼 좌우 착지 강도 차로
// 스트라이드 주기성이 스텝 주기성과 비슷해지는 신호에서 스텝/스트라이드 2배 모호를 스텝 쪽으로 해소(실측 2026-09-27: r15 .745 vs r30 .736)
function acfPeriod(sig, fs, fmin, fmax, tol, sub){ const n=sig.length, T=(tol==null? 0.03 : tol);
  const lmin=Math.max(2,Math.ceil(fs/fmax)), lmax=Math.min(n-3,Math.floor(fs/fmin));
  if(lmax-lmin<2) return {f0:null, strength:0, lag:null};
  const score=l=>{ let ab=0,aa=0,bb=0; for(let i=0;i+l<n;i++){ ab+=sig[i]*sig[i+l]; aa+=sig[i]*sig[i]; bb+=sig[i+l]*sig[i+l]; } return aa*bb>1e-12? ab/Math.sqrt(aa*bb) : 0; };
  const r=new Float64Array(lmax+2); for(let l=lmin-1;l<=lmax+1;l++) r[l]=score(l);
  // 국소 피크마다 포물선 보간(위치·높이). 높이는 보간값으로 비교(피크가 샘플 사이에 있으면 샘플값이 낮아 짧은 주기가 부당하게 탈락)
  const peaks=[]; for(let l=lmin;l<=lmax;l++) if(r[l]>=r[l-1] && r[l]>=r[l+1]){ const a=r[l-1], b=r[l], c=r[l+1], den=a-2*b+c;
    const ok=Math.abs(den)>1e-9, dl=ok? Math.max(-.5,Math.min(.5,.5*(a-c)/den)) : 0, y=ok? Math.min(1, b-0.125*(a-c)*(a-c)/den) : b; peaks.push({l:l+dl, y}); }
  if(!peaks.length) return {f0:null, strength:0, lag:null};
  let best=-2; for(const p of peaks) if(p.y>best) best=p.y;
  let p=peaks.find(p=>p.y>=best-T);
  if(sub>0){ const h=p.l/2, q=peaks.filter(q=>Math.abs(q.l-h)<=0.2*h && q.y>=sub*p.y && fs/q.l<=fmax).sort((a,b)=>b.y-a.y)[0]; if(q) p=q; }
  const f0=fs/p.l; if(f0<fmin||f0>fmax) return {f0:null, strength:Math.max(0,p.y), lag:p.l};
  return {f0, strength:Math.max(0,p.y), lag:p.l}; }
// Hann 창 Goertzel 진폭(창 합으로 정규화). f 가 유효범위 밖이면 null
function goertzelHann(sig, fs, f){ if(!(f>0 && f<fs/2) || sig.length<3) return null; const n=sig.length, w=2*Math.PI*f/fs, c=2*Math.cos(w); let s1=0,s2=0,ws=0;
  for(let i=0;i<n;i++){ const h=0.5-0.5*Math.cos(2*Math.PI*i/(n-1)); ws+=h; const s0=sig[i]*h+c*s1-s2; s2=s1; s1=s0; }
  return 2*Math.hypot(s1-s2*Math.cos(w), s2*Math.sin(w))/ws; }
// 고조파 진폭비(진단용) E = 100·A(2f0)/A(f0). 기본파가 약하면(minFundFrac·std 미만) null. 클램프 없음(100 초과 가능)
function harmonicRatio(sig, fs, f0, std, minFundFrac){ const m1=goertzelHann(sig,fs,f0), m2=goertzelHann(sig,fs,2*f0);
  if(m1==null||m2==null||!(m1>minFundFrac*std)) return null; return 100*m2/m1; }
// 선회 게이트(yaw_rate): 추세제거 전 신호의 창 평균 |mean|>meanDps 또는 segSec 구간(1s hop) 평균 >segDps → hold
function turnGate(rawSig, fs, meanDps, segDps, segSec){ const m=mean(rawSig), blk=Math.round(segSec*fs), hop=Math.round(fs); let segMax=0;
  for(let i=0;i+blk<=rawSig.length;i+=hop){ let s=0; for(let k=i;k<i+blk;k++) s+=rawSig[k]; segMax=Math.max(segMax, Math.abs(s/blk)); }
  return {hold: Math.abs(m)>meanDps || segMax>segDps, meanDps:m, segMaxDps:segMax}; }
// 진폭 급변 게이트: 창 앞 ⅓ vs 뒤 ⅓ 표준편차 비 > ratio → hold (가속·감속 구간)
function rampGate(sig, ratio){ const n3=Math.floor(sig.length/3); if(n3<10) return {hold:false, ratio:null};
  const a=stdOf(sig.slice(0,n3)), b=stdOf(sig.slice(sig.length-n3)), r=Math.max(a,b)/Math.max(1e-6,Math.min(a,b)); return {hold:r>ratio, ratio:r}; }
// ---------------------------------------------------------------- 로브 ----
// 로브 추출(규칙 고정 순서). sig: 추세제거 격자 신호, g: 진입 임계. o={exitFrac, minLen, flagged, flatRatio, minRel}
//  ① 진입 |x|>g · 이탈 |x|<exitFrac·g 또는 부호 반전(히스테리시스), 로브당 극값 1개(최대 |x|, 동률이면 첫 샘플)
//  ② 동부호 인접 로브 병합(M자 딥)  ③ short(<minLen 샘플, 스파이크) 제거 후 재병합  ④ gapped(극값이 결손 보간점)
//  ⑤ flat(극값 < flatRatio × 로브 평균|x|: 극값 불명확)  ⑥ weak(극값 < minRel × 중앙 극값)  ⑦ 양끝 edge(창에 잘림 가능)
// 반환 {lobes:[{sign,start,end,i,v,role}] (short 포함, 시간순), kept:[역할 없는 짝 후보]}
function lobes(sig, g, o){ const exitG=g*o.exitFrac, flagged=o.flagged||null; const L=[]; let cur=null;
  for(let i=0;i<sig.length;i++){ const x=sig[i], ax=Math.abs(x), sg=x>0?1:x<0?-1:0;
    if(cur){ if(sg===cur.sign && ax>=exitG){ cur.end=i; if(ax>cur.v){ cur.v=ax; cur.i=i; } continue; } cur=null; }
    if(ax>g){ cur={sign:sg, start:i, end:i, i, v:ax, role:null, merged:0}; L.push(cur); } }
  const merge=arr=>{ const out=[]; for(const lb of arr){ const last=out[out.length-1];
      if(last && last.sign===lb.sign){ if(lb.v>last.v){ last.v=lb.v; last.i=lb.i; } last.end=lb.end; last.merged++; } else out.push(lb); } return out; };
  let k=merge(L);
  for(const lb of k) if(lb.end-lb.start+1<o.minLen) lb.role='short';
  const shorts=k.filter(lb=>lb.role==='short'); k=merge(k.filter(lb=>!lb.role));
  for(const lb of k){ if(flagged && flagged[lb.i]){ lb.role='gapped'; continue; }
    let s=0; for(let j=lb.start;j<=lb.end;j++) s+=Math.abs(sig[j]); if(lb.v < o.flatRatio*(s/(lb.end-lb.start+1))) lb.role='flat'; }
  const med=median(k.filter(lb=>!lb.role).map(lb=>lb.v));
  if(med!=null) for(const lb of k) if(!lb.role && lb.v<o.minRel*med) lb.role='weak';
  const cand=k.filter(lb=>!lb.role);
  if(cand.length){ cand[0].role='edge'; cand[cand.length-1].role='edge'; }
  return {lobes:k.concat(shorts).sort((a,b)=>a.i-b.i), kept:cand.filter(lb=>lb.role!=='edge')}; }
// 로브 교대 주기(샘플): 동부호 로브(한 로브 건너) 극값 간격의 중앙값. 간격 <3개 → null
function lobeAlternationT(kept){ const gaps=[]; for(let i=2;i<kept.length;i++) if(kept[i].sign===kept[i-2].sign && kept[i-1].sign!==kept[i].sign) gaps.push(kept[i].i-kept[i-2].i);
  return gaps.length>=3? median(gaps) : null; }
// 스트라이드 주파수 교차검증: 후보(acc·metric·lobe) 중앙값. tol 초과 이탈 후보를 outliers 로, 후보 간 전부 불일치면 ambiguous
function resolveStride(c, tol){ const cands=[['acc',c.fAcc],['metric',c.fMetric],['lobe',c.fLobe]].filter(x=>x[1]>0);
  if(!cands.length) return {f:null, src:null, ambiguous:true, outliers:[]};
  const f=median(cands.map(x=>x[1])), outliers=cands.filter(x=>Math.abs(x[1]-f)/f>tol).map(x=>x[0]);
  const hit=cands.find(x=>x[1]===f), src=cands.length===1? cands[0][0] : (hit? hit[0] : 'mean');
  return {f, src, ambiguous: cands.length>=2 && outliers.length>=cands.length-1, outliers}; }
// 짝짓기: 후보 로브를 시간순으로, 인접 반대부호·극값 간격 ∈ [pairMin,pairMax]·T 이면 짝(pos,neg). 실패 로브 role=unpaired
function pairLobes(kept, T, pairMin, pairMax){ const pairs=[]; let i=0;
  while(i+1<kept.length){ const a=kept[i], b=kept[i+1], gap=b.i-a.i;
    if(a.sign!==b.sign && gap>=pairMin*T && gap<=pairMax*T){ a.role='paired'; b.role='paired'; pairs.push({pos:a.sign>0?a:b, neg:a.sign>0?b:a}); i+=2; } else i++; }
  for(const lb of kept) if(!lb.role) lb.role='unpaired';
  return pairs; }
// 부호별 절사 대표값: 쌍 ≥ trimMinPairs 이면 각 쪽 상·하위 trimK 개 절사 후 평균, 아니면 중앙값. L/R = 양/음 로브 극값 대표(크기)
// outliers: 절사(또는 중앙값 대비 개별) 값이 대표값의 ×1.5 초과 / ×0.5 미만인 로브 — 이상 피크 존재 표시 + 0 레벨 보정에 사용
function trimmedSI(pairs, trimK, trimMinPairs){ if(!pairs.length) return {L:null, R:null, trimmedCount:0, outlierFlag:false, outliers:[], method:null};
  const useTrim=pairs.length>=trimMinPairs;
  const agg=arr=>{ const s=arr.slice().sort((a,b)=>a.v-b.v); if(useTrim){ return {m:mean(s.slice(trimK, s.length-trimK).map(l=>l.v)), removed:s.slice(0,trimK).concat(s.slice(s.length-trimK))}; } return {m:median(s.map(l=>l.v)), removed:s}; };
  const a=agg(pairs.map(p=>p.pos)), b=agg(pairs.map(p=>p.neg)), L=a.m, R=b.m;
  const isOut=(l,m)=> l.v>1.5*m || l.v<0.5*m;
  const outliers=a.removed.filter(l=>isOut(l,L)).concat(b.removed.filter(l=>isOut(l,R)));
  return {L, R, trimmedCount: useTrim? 4*trimK : 0, outlierFlag: outliers.length>0, outliers, method: useTrim?'trim':'median'}; }
// 이상 로브의 초과 면적이 평균제거(0 레벨)를 밀어낸 만큼 되돌림: 이상 로브를 대표 극값 크기로 축소했을 때의 평균 = 정상 신호의 0 레벨.
// 반환 shift(신호 단위). 보정 후 L' = L − shift, R' = R + shift
function zeroShiftFromOutliers(sig, outliers, L, R){ let extra=0;
  for(const l of outliers){ const m=(l.sign>0? L : R); let s=0; for(let j=l.start;j<=l.end;j++) s+=sig[j]; extra+=s*(1-m/l.v); }
  return sig.length? -extra/sig.length : 0; }
// 로브 → 표시/기록용 피크 목록 {i,t,v(부호있음),sign,role,start,end}
function peaksOf(lobeList, sig, tGrid0, fs){ const dt=1000/fs; return lobeList.map(l=>({i:l.i, t:tGrid0+l.i*dt, v:sig[l.i], sign:l.sign, role:l.role||'unpaired', start:l.start, end:l.end})); }

// ------------------------------------------------------------ 시간축 ----
// 도착시각 보정(de-burst): 뒤에서부터 t̂_k = min(t_k, t̂_{k+1} − Δ). Δ 는 명목 주기(기기는 명목보다 빠를 수 없음).
// 평균 간격이 명목의 90% 미만이면(fsGrid 설정이 기기보다 느림) 평균으로 폴백해 누적 드리프트를 막음. 반환은 단조 증가(간격 ≥ Δ).
function reconstructTimes(t, dNomMs){ const n=t.length, th=new Float64Array(n); if(!n) return th;
  const avg=(n>1)? (t[n-1]-t[0])/(n-1) : dNomMs, d=(avg<0.9*dNomMs)? avg : dNomMs;
  th[n-1]=t[n-1]; for(let k=n-2;k>=0;k--) th[k]=Math.min(t[k], th[k+1]-d);
  return th; }
// 실제 수신 주기 추정(ms): 도착 간격의 중앙값을 [Δnom, 3Δnom] 로 클램프. 버스트 도착(간격≈0)은 명목으로, 느린 기기/링크(예: 32Hz)는 실제 주기로.
// 결손(드롭)은 소수라 중앙값에 영향이 작음. 격자(fsGrid)는 그대로 두고 de-burst·coverage·오일러 미분에만 사용
function effectivePeriod(t, dNom){ if(t.length<3) return dNom; const g=new Array(t.length-1); for(let i=1;i<t.length;i++) g[i-1]=t[i]-t[i-1];
  return Math.max(dNom, Math.min(3*dNom, median(g))); }
// 결손 통계: 최대 간격, coverage = 1 − Σ_{gap>1.5Δ}(gap−Δ)/창길이
function gapStats(th, winMs, d){ let maxGap=0, excess=0;
  for(let k=1;k<th.length;k++){ const g=th[k]-th[k-1]; if(g>maxGap) maxGap=g; if(g>1.5*d) excess+=g-d; }
  return {maxGapMs:maxGap, coverage:Math.max(0, 1-excess/Math.max(1,winMs))}; }
// 균일 격자 선형보간(외삽 없음). 넓은 결손 위 격자점은 flagged=1.
function resample(th, v, t0, dt, n, flagGapMs){ const y=new Float64Array(n), flagged=new Uint8Array(n); const m=th.length; let j=0;
  for(let k=0;k<n;k++){ const tk=t0+k*dt;
    if(tk<=th[0]){ y[k]=v[0]; flagged[k]=tk<th[0]?1:0; continue; }
    while(j+1<m && th[j+1]<tk) j++;
    if(j+1>=m){ y[k]=v[m-1]; flagged[k]=1; continue; }
    const a=th[j], b=th[j+1], f=(b>a)?(tk-a)/(b-a):0; y[k]=v[j]+f*(v[j+1]-v[j]); if(b-a>flagGapMs) flagged[k]=1; }
  return {y, flagged}; }
function sanityFilter(samples, maxAbs){ const kept=[]; let nInvalid=0;
  for(const s of samples){ if(!Number.isFinite(s.v)||Math.abs(s.v)>maxAbs){ nInvalid++; continue; } kept.push(s); }
  return {kept, nInvalid}; }
// 레벨 시프트(각도): |Δ|>jump 이고 이후 minSamples 가 이전 레벨에서 jump/2 이상 떨어져 유지 → 인덱스, 없으면 −1
function referenceJump(v, jumpDeg, minSamples){
  for(let i=1;i<v.length;i++){ if(Math.abs(v[i]-v[i-1])>jumpDeg){ const pre=v[i-1]; let hold=0;
      for(let k=i;k<v.length && k<i+minSamples;k++) if(Math.abs(v[k]-pre)>jumpDeg/2) hold++;
      if(hold>=minSamples) return i; } }
  return -1; }
// 도착 샘플 {t,v} → 창 [t0,t1] 균일 격자(평균 제거 옵션). 내부 공용. 실패 시 null
function gridOf(samples, t0, t1, fs, removeMean){ const dt=1000/fs; if(samples.length<10) return null;
  const tArr=samples.map(s=>s.t), v=samples.map(s=>s.v), th=reconstructTimes(tArr, effectivePeriod(tArr, dt));
  const g0=Math.max(t0, th[0]), gEnd=Math.min(t1, th[th.length-1]), n=Math.floor((gEnd-g0)/dt)+1; if(n<10) return null;
  const y=Array.from(resample(th, v, g0, dt, n, 3*dt).y); if(removeMean){ const m=mean(y); for(let i=0;i<n;i++) y[i]-=m; }
  return {y, fs, t0:g0, n}; }

// -------------------------------------------------------- RawEngine ----
// 원시 IMU 버퍼 {t(ms), g[3] dps(센서프레임, bias 미차감), a[3] g}. 45 s 유지.
class RawEngine {
  constructor(opts){ this.buf=[]; this.cfg=gateWith(opts&&opts.gate); this.bias=[0,0,0]; this.keepMs=(opts&&opts.keepMs)||45000;
    this._st={running:false, passT:null, failT:null}; this._lastWinRunning=false; }
  push(t, g, a){ this.buf.push({t, g:[g[0],g[1],g[2]], a:[a[0],a[1],a[2]]}); if(this.keepMs<Infinity){ const cut=t-this.keepMs; while(this.buf.length && this.buf[0].t<cut) this.buf.shift(); } }
  alive(t, maxAgeMs){ const n=this.buf.length; return n>0 && (t-this.buf[n-1].t) <= (maxAgeMs==null? 500 : maxAgeMs); }
  slice(t0,t1){ return this.buf.filter(s=>s.t>=t0 && s.t<=t1); }
  setBias(b){ this.bias=[b[0],b[1],b[2]]; }
  // 창 평균 가속도 방향 = 중력-up(센서프레임). 러닝의 동적가속도는 여러 스트라이드 평균에서 상쇄
  upFromAccelMean(t0,t1){ const s=this.slice(t0,t1); if(s.length<5) return null; const m=[0,0,0];
    for(const x of s){ m[0]+=x.a[0]; m[1]+=x.a[1]; m[2]+=x.a[2]; } const n=Math.hypot(m[0],m[1],m[2]); return n<1e-6? null : [m[0]/n,m[1]/n,m[2]/n]; }
  // 정지 구간 자이로 bias 추정(|a| 표준편차가 작을 때만 ok)
  estimateBias(t0,t1){ const s=this.slice(t0,t1); if(s.length<10) return {ok:false, n:s.length, stdG:null, bias:null};
    const stdG=stdOf(s.map(x=>Math.hypot(x.a[0],x.a[1],x.a[2]))); const b=[0,0,0]; for(const x of s){ b[0]+=x.g[0]; b[1]+=x.g[1]; b[2]+=x.g[2]; }
    for(let k=0;k<3;k++) b[k]/=s.length; return {ok: stdG<this.cfg.acc.stationaryStdG, n:s.length, stdG, bias:b}; }
  // 수직축 각속도 샘플 {t, v}(dps): dot(g−bias, up)
  vertRateSamples(t0,t1,up){ const b=this.bias; return this.slice(t0,t1).map(x=>({t:x.t, v:yawRate(x.g, up, b)})); }
  // |a| 노름 샘플
  accNormSamples(t0,t1){ return this.slice(t0,t1).map(x=>({t:x.t, v:Math.hypot(x.a[0],x.a[1],x.a[2])})); }
  // 창 통계: 1초 구간 중 정지(std|a| < stationaryStdG) 비율
  stats(t0,t1){ const s=this.accNormSamples(t0,t1); if(s.length<10) return {stationaryFrac:null, nSeg:0};
    let nSeg=0, nStat=0; for(let a=t0; a+1000<=t1+1; a+=1000){ const seg=[]; for(const x of s) if(x.t>=a && x.t<a+1000) seg.push(x.v);
      if(seg.length<10) continue; nSeg++; if(stdOf(seg)<this.cfg.acc.stationaryStdG) nStat++; }
    return {stationaryFrac: nSeg? nStat/nSeg : null, nSeg}; }
  // |a| 스텝 케이던스: 평균 제거 격자 → 상관계수 ACF(stepMin~stepMax) → fStep(Hz), strength, stdG(g)
  stepCadence(t0,t1){ const c=this.cfg; const g=gridOf(this.accNormSamples(t0-100,t1+100), t0, t1, c.fsGrid, true);
    if(!g) return {fStep:null, strength:0, stdG:null};
    const stdG=Math.sqrt(g.y.reduce((s,v)=>s+v*v,0)/g.n); const p=acfPeriod(g.y, g.fs, c.acc.stepMin, c.acc.stepMax, c.peakTol, c.acc.subHarm);
    return {fStep:p.f0, strength:p.strength, stdG}; }
  // 창 [t0,t1] 러닝 판정: blockSec 블록(hop hopSec)마다 (stdG>임계 & 스텝대역 주기성) → 통과 블록 비율 ≥ blockFrac.
  // 직전 창이 러닝이면 exitStdG(히스테리시스) 적용.
  windowRunning(t0,t1){ const c=this.cfg.acc, blk=c.blockSec*1000, hop=c.hopSec*1000, thr=this._lastWinRunning? c.exitStdG : c.enterStdG;
    let nb=0, np=0; const spm=[], stds=[];
    for(let s=t0; s+blk<=t1+1; s+=hop){ const r=this.stepCadence(s,s+blk); nb++; if(r.stdG!=null) stds.push(r.stdG);
      if(r.stdG!=null && r.stdG>thr && r.fStep!=null && r.strength>c.minStrength){ np++; spm.push(60*r.fStep); } }
    const frac=nb? np/nb : 0, running=nb>0 && frac>=c.blockFrac; this._lastWinRunning=running;
    return {running, frac, nBlocks:nb, stepSpm: spm.length? median(spm) : null, stdG: stds.length? median(stds) : null}; }
  // 라이브 표시용 히스테리시스 상태기계(hopSec 마다 호출): enterSec 연속 통과 → running, exitSec 연속 실패 → 해제
  tick(tNow){ const c=this.cfg.acc, r=this.stepCadence(tNow-c.blockSec*1000, tNow), st=this._st;
    const thr=st.running? c.exitStdG : c.enterStdG, pass=r.stdG!=null && r.stdG>thr && r.fStep!=null && r.strength>c.minStrength;
    if(pass){ st.failT=null; if(st.passT==null) st.passT=tNow; if(!st.running && tNow-st.passT>=c.enterSec*1000) st.running=true; }
    else { st.passT=null; if(st.failT==null) st.failT=tNow; if(st.running && tNow-st.failT>=c.exitSec*1000) st.running=false; }
    return {running:st.running, stdG:r.stdG, fStep:r.fStep, stepSpm:(r.fStep? 60*r.fStep : null), strength:r.strength}; }
}

// ------------------------------------------------------------- 엔진 ----
// 자세 버퍼 원소 {t(ms), roll(오일러), yaw, lean(사람기준), up[3]?} — t 는 호출자가 주입(앱: performance.now)
class AsymEngine {
  constructor(opts){ this.buf=[]; this.cfg=gateWith(opts&&opts.gate); this.raw=(opts&&opts.raw)||null; this.keepMs=(opts&&opts.keepMs)||45000; }
  push(t, roll, yaw, lean, up){ this.buf.push({t,roll,yaw,lean,up:up||null}); if(this.keepMs<Infinity){ const cut=t-this.keepMs; while(this.buf.length && this.buf[0].t<cut) this.buf.shift(); } }
  // 지표별 원천 샘플 {t, v}. 'roll'/'lean' = 사람기준 lean(°). 'yaw_rate' = raw 자이로 수직축 투영(dps), raw 없으면 오일러 yaw 차분/실제 수신주기
  // tFrom(선택): 샘플 시작 시각(표시용 사전구간 포함 시). up 은 항상 분석 창 [t0,t1] 평균
  _source(metric, t0, t1, tFrom){
    if(metric==='yaw_rate'){
      if(this.raw && this.raw.alive(t1, 1000)){ const up=this.raw.upFromAccelMean(t0, t1);
        if(up) return {samples:this.raw.vertRateSamples(tFrom==null? t0-100 : Math.min(tFrom, t0-100), t1+100, up), kind:'dps', srcYaw:'raw', up}; }
      const b=this.buf, win=b.filter(s=>s.t>=t0&&s.t<=t1), dtS=effectivePeriod(win.map(s=>s.t), 1000/this.cfg.fsGrid)/1000, out=[];
      for(let i=1;i<b.length;i++){ let dy=b[i].yaw-b[i-1].yaw; if(dy>180)dy-=360; else if(dy<-180)dy+=360; out.push({t:b[i].t, v:dy/dtS}); }
      return {samples:out, kind:'dps', srcYaw:'euler', up:null}; }
    return {samples:this.buf.map(s=>({t:s.t, v:s.lean})), kind:'deg', srcYaw:null, up:null};
  }
  // 창 [t0,t1] 의 지표 신호: 시간범위·정합성·stale·결손 검사 → de-burst → 균일 리샘플 → (각도) 레벨시프트 → detrend
  windowSig(metric, t0, t1, tNow){
    const c=this.cfg, winMs=t1-t0, dt=1000/c.fsGrid, src=this._source(metric, t0, t1);
    const fail=(reason, extra)=>Object.assign({hold:{reason}, srcYaw:src.srcYaw, up:src.up, coverage:null, gapMaxMs:null, dEff:null}, extra||{});
    const rawS=src.samples.filter(s=>s.t>=t0-2*dt && s.t<=t1+2*dt);
    if(rawS.length<4) return fail('insufficient_duration');
    const {kept, nInvalid}=sanityFilter(rawS, src.kind==='dps'? c.maxAbsDps : c.maxAbsDeg);
    if(nInvalid>0.05*rawS.length || kept.length<4) return fail('sample_invalid');
    const tArr=kept.map(s=>s.t), vArr=kept.map(s=>s.v);
    if(Number.isFinite(tNow) && tNow-tArr[tArr.length-1] > c.staleMs) return fail('stale_data');
    const dEff=effectivePeriod(tArr, dt);                 // 실제 수신 주기(느린 링크면 > 명목) — de-burst·결손 계산 기준
    const th=reconstructTimes(tArr, dEff), span=th[th.length-1]-th[0];
    if(span < c.minSpanFrac*winMs) return fail('insufficient_duration', {dEff});
    const gs=gapStats(th, winMs, dEff);
    if(gs.maxGapMs > c.maxGapMs) return fail('sample_gap', {coverage:gs.coverage, gapMaxMs:gs.maxGapMs, dEff});
    const g0=Math.max(t0, th[0]), gEnd=Math.min(t1, th[th.length-1]), n=Math.floor((gEnd-g0)/dt)+1;
    if(n<20) return fail('insufficient_duration', {dEff});
    const rs=resample(th, vArr, g0, dt, n, 3*dEff), y=Array.from(rs.y);
    if(src.kind==='deg' && referenceJump(y, c.refJumpDeg, c.refJumpMinSamples)>=0) return fail('reference_jump', {coverage:gs.coverage, gapMaxMs:gs.maxGapMs, dEff});
    const meanOffset=mean(y), sig=y.slice(); detrend(sig);
    return {hold:null, sig, raw:y, fs:c.fsGrid, tGrid0:g0, n, coverage:gs.coverage, gapMaxMs:gs.maxGapMs, dEff, meanOffset, flagged:rs.flagged, srcYaw:src.srcYaw, up:src.up};
  }
  // 상태 객체 R. 파이프라인: 입력품질(hold) → (yaw)선회 → 램프 → 정지비율 → 러닝판정(acc; not_running) → 진폭(info) → 주기성(info)
  //   → 로브 추출 → 스트라이드 교차검증(hold period_ambiguous) → 로브 개수(hold lobe_count) → 짝(info unclear_alternation / hold data_gaps)
  //   → 절사 SI → E → (lobe 주기 이탈 info) → (|SI|>cap info) → ok
  analyzeRange(metric, t0, t1, tNow){
    const c=this.cfg;
    const R={ state:'hold', reason:null, metric, t0, t1, winSec:(t1-t0)/1000, running:false, runSrc:'metric', accStdG:null, stationaryFrac:null,
      stepSpm:null, stepSrc:null, strideHz:null, strideSrc:'metric', strideCands:null, metricHz:null, strength:null, std:null, meanOffset:null, rampRatio:null, turnMeanDps:null, turnSegMaxDps:null,
      gate:null, nLobes:0, nPairs:0, meanL:null, meanR:null, signedSI:null, si:null, side:null, trimmedCount:0, outlierFlag:false, zeroShift:0, siMethod:null, E:null,
      coverage:null, gapMaxMs:null, devHz:null, srcYaw:null, upMismatchDeg:null, peaks:[], sig:null, tGrid0:null, fs:c.fsGrid, engine:ENGINE_VER };
    let lb=null;   // 로브 추출 결과(있으면 done() 에서 peaks 로 기록)
    const done=()=>{ if(lb) R.peaks=peaksOf(lb.lobes, R.sig, R.tGrid0, R.fs); return legacyAliases(R); };
    const ws=this.windowSig(metric, t0, t1, tNow);
    R.srcYaw=ws.srcYaw; R.coverage=ws.coverage; R.gapMaxMs=ws.gapMaxMs; R.devHz=ws.dEff? 1000/ws.dEff : null;
    if(ws.hold){ R.reason=ws.hold.reason; return done(); }
    const {sig, fs}=ws, isRate=(metric==='yaw_rate');
    R.sig=sig; R.tGrid0=ws.tGrid0; R.meanOffset=ws.meanOffset;
    // 쿼터니언-up vs 가속도평균-up 불일치(진단)
    if(ws.up){ const ups=this.buf.filter(s=>s.t>=t0&&s.t<=t1&&s.up); if(ups.length){ const m=[0,0,0]; for(const s of ups){ m[0]+=s.up[0]; m[1]+=s.up[1]; m[2]+=s.up[2]; }
      const n=Math.hypot(m[0],m[1],m[2]); if(n>1e-6) R.upMismatchDeg=angleDeg([m[0]/n,m[1]/n,m[2]/n], ws.up); } }
    // (yaw_rate) 선회 오염: 추세제거 전 신호
    if(isRate){ const tg=turnGate(ws.raw, fs, c.turnMeanDps, c.turnSegDps, c.turnSegSec); R.turnMeanDps=tg.meanDps; R.turnSegMaxDps=tg.segMaxDps;
      if(tg.hold){ R.reason='turn_contamination'; return done(); } }
    const std=Math.sqrt(sig.reduce((s,v)=>s+v*v,0)/sig.length); R.std=std;
    // 진폭 급변(가속·감속)
    const rg=rampGate(sig, c.rampRatio); R.rampRatio=rg.ratio; if(rg.hold){ R.reason='amplitude_ramp'; return done(); }
    // 러닝 판정(가속도) — 정지 구간 비율, 블록 러닝 비율
    const hasRaw = !!(this.raw && this.raw.alive(t1, 1000));
    if(hasRaw){ R.runSrc='acc'; const st=this.raw.stats(t0,t1); R.stationaryFrac=st.stationaryFrac;
      if(st.stationaryFrac!=null && st.stationaryFrac>c.stationaryMaxFrac){ R.state='not_running'; R.reason='stationary'; return done(); }
      const wr=this.raw.windowRunning(t0,t1); R.running=wr.running; R.stepSpm=wr.stepSpm; R.accStdG=wr.stdG;
      if(wr.stepSpm){ R.strideHz=wr.stepSpm/120; R.strideSrc='acc'; }
      if(!wr.running){ R.state='not_running'; R.reason='not_running'; return done(); } }
    // 진폭 하한 → 정보(안정)
    if(std < (isRate? c.minRateStd : c.minAngleStd)){ R.state='info'; R.reason='low_amplitude'; return done(); }
    // 케이던스·주기성(지표 신호) → 정보(불규칙)
    const p=acfPeriod(sig, fs, c.cadMin, c.cadMax, c.peakTol); R.strength=p.strength; R.metricHz=p.f0;
    if(!p.f0 || p.strength<c.minStrength){ R.state='info'; R.reason='weak_periodicity'; return done(); }
    if(!hasRaw){ R.running=true; R.runSrc='metric'; }
    // 로브 추출(예비 주기 = 지표 ACF — |a| 케이던스는 2배 모호가 남을 수 있어 로브 최소 길이에 쓰지 않음)
    const fAcc=(R.strideSrc==='acc')? R.strideHz : null, fPre=p.f0;
    R.gate=Math.max(c.lobeK*std, isRate? c.lobeFloorDps : c.lobeFloorDeg);
    lb=lobes(sig, R.gate, {exitFrac:c.lobeExitFrac, minLen:Math.max(c.minLobeSamples, Math.round(c.minLobeFrac*fs/fPre)), flagged:ws.flagged, flatRatio:c.flatRatio, minRel:c.minLobeRel});
    R.nLobes=lb.lobes.filter(l=>l.role!=='short').length;   // 개수 타당성은 품질 역할과 무관하게 진동 로브 수로
    // 스트라이드 교차검증: acc · metric ACF · 로브 교대 의 중앙값
    const tLobe=lobeAlternationT(lb.kept), fLobe=tLobe? fs/tLobe : null;
    const rs=resolveStride({fAcc, fMetric:p.f0, fLobe}, c.tDisagree);
    R.strideCands={acc:fAcc, metric:p.f0, lobe:fLobe}; R.strideHz=rs.f; R.strideSrc='resolved:'+rs.src;
    if(rs.ambiguous){ R.reason='period_ambiguous'; return done(); }
    if(fAcc!=null && rs.outliers.includes('acc')){ R.stepSpm=120*rs.f; R.stepSrc='derived'; }   // |a| 케이던스가 2배 틀리면 확정 스트라이드에서 유도
    else if(R.stepSpm!=null) R.stepSrc='acc';
    const T=fs/rs.f, nExp=2*R.winSec*rs.f;
    if(R.nLobes<c.lobeCountLo*nExp || R.nLobes>c.lobeCountHi*nExp){ R.reason='lobe_count'; return done(); }
    // 짝짓기 → 부족하면 결손 기인(hold) / 교대 불명확(info)
    const pairs=pairLobes(lb.kept, T, c.pairMin, c.pairMax); R.nPairs=pairs.length;
    if(pairs.length<c.minPairs){ const gapped=lb.lobes.filter(l=>l.role==='gapped').length;
      if((R.coverage!=null && R.coverage<0.9) || gapped>=2){ R.reason='data_gaps'; return done(); }
      R.state='info'; R.reason='unclear_alternation'; return done(); }
    // 절사 대표값 → (이상 로브 있으면 0 레벨 보정) → signedSI. 고조파 진폭비(진단)
    const S=trimmedSI(pairs, c.trimK, c.trimMinPairs);
    R.zeroShift = S.outliers.length? zeroShiftFromOutliers(sig, S.outliers, S.L, S.R) : 0;
    const L=S.L-R.zeroShift, Rm=S.R+R.zeroShift, signedSI=(L+Rm)>1e-9? (L-Rm)/(L+Rm)*100 : 0;
    Object.assign(R, { meanL:L, meanR:Rm, signedSI, si:Math.abs(signedSI), side:(signedSI>=0?'좌':'우'), trimmedCount:S.trimmedCount, outlierFlag:S.outlierFlag, siMethod:S.method });
    R.E=harmonicRatio(sig, fs, rs.f, std, c.eMinFundFrac);
    if(rs.outliers.includes('lobe')){ R.state='info'; R.reason='period_ambiguous'; return done(); }
    if(R.si>c.siCapPct){ R.state='info'; R.reason='unclear_alternation'; return done(); }
    R.state='ok'; R.reason=null; return done();
  }
  analyze(winSec, metric, method, tNow){
    if(this.buf.length<2) return legacyAliases({state:'hold', reason:'insufficient_duration', metric, winSec, running:false, si:null, signedSI:null, side:null, nPairs:0, E:null, strideHz:null, stepSpm:null, coverage:null, peaks:[], engine:ENGINE_VER});
    const tLast=this.buf[this.buf.length-1].t;
    return this.analyzeRange(metric, tLast-winSec*1000, tLast, tNow);
  }
  // 정지 스냅샷: 분석 범위 [aT0,aT1] 의 R(stale 검사 없음) + 표시 격자 [showT0,showT1](사전구간 포함, 평균오프셋 제거·분석구간은 R.sig)
  // 반환 {metric, srcYaw, R, disp:{t0,y,dt}, span, aStart, aStop, peaks:[{...,tRel}], gate} | null
  snapshot(metric, rng){
    const c=this.cfg, dt=1000/c.fsGrid;
    const R=this.analyzeRange(metric, rng.aT0, rng.aT1, undefined);
    const src=this._source(metric, rng.aT0, rng.aT1, rng.showT0-2*dt);
    const w=src.samples.filter(s=> s.t>=rng.showT0-2*dt && s.t<=rng.showT1+2*dt && Number.isFinite(s.v));
    const g=gridOf(w, rng.showT0, rng.showT1, c.fsGrid, false);
    if(!g && !R.sig) return null;
    let disp;
    if(g){ const off=(R.meanOffset!=null)? R.meanOffset : mean(g.y), y=g.y.map(v=>v-off);
      if(R.sig){ const k0=Math.round((R.tGrid0-g.t0)/dt); for(let k=0;k<R.sig.length;k++){ const j=k0+k; if(j>=0 && j<y.length) y[j]=R.sig[k]; } }
      disp={t0:g.t0, y, dt}; }
    else disp={t0:R.tGrid0, y:R.sig.slice(), dt};
    // 상태와 무관한 표시용 로브(게이트 조기 종료로 로브가 없을 때): 예비 주기로 추출·짝만
    if(!R.peaks.length && R.sig){ const f=R.strideHz||R.metricHz||acfPeriod(R.sig, R.fs, c.cadMin, c.cadMax, c.peakTol).f0;
      if(f){ const isRate=(metric==='yaw_rate'), std=R.std!=null? R.std : Math.sqrt(R.sig.reduce((s,v)=>s+v*v,0)/R.sig.length);
        const gate=Math.max(c.lobeK*std, isRate? c.lobeFloorDps : c.lobeFloorDeg);
        const lb=lobes(R.sig, gate, {exitFrac:c.lobeExitFrac, minLen:Math.max(c.minLobeSamples, Math.round(c.minLobeFrac*R.fs/f)), flagged:null, flatRatio:c.flatRatio, minRel:c.minLobeRel});
        pairLobes(lb.kept, R.fs/f, c.pairMin, c.pairMax); R.peaks=peaksOf(lb.lobes, R.sig, R.tGrid0, R.fs); if(R.gate==null) R.gate=gate; } }
    return {metric, srcYaw:R.srcYaw, R, disp, span:rng.showT1-disp.t0, aStart:rng.aT0-disp.t0, aStop:rng.aT1-disp.t0,
            peaks:R.peaks.map(p=>Object.assign({tRel:p.t-disp.t0}, p)), gate:R.gate};
  }
}
// ------------------------------------------------------------ 기준선 ----
// 사전 측정 고정 baseline: ok 창의 signedSI 분포(전체 + 케이던스 구간별). summary() 는 JSON 직렬화 가능한 순수 객체(schema 1)
class BaselineStats {
  constructor(meta){ this.meta=Object.assign({schema:1, metric:null, forwardAxis:null, signFlip:false, deviceId:null, engineVer:ENGINE_VER, winSec:GATE.winSec, binSpm:GATE.baseline.binSpm}, meta||{}); this.rows=[]; }
  add(R){ if(!R || R.state!=='ok' || typeof R.signedSI!=='number') return false;
    this.rows.push({si:R.signedSI, E:(R.E==null? null : R.E), spm:(R.stepSpm==null? null : R.stepSpm), off:(R.meanOffset==null? null : R.meanOffset)}); return true; }
  get n(){ return this.rows.length; }
  complete(minWindows){ return this.rows.length >= (minWindows==null? GATE.baseline.minWindows : minWindows); }
  summary(dateMs){ const si=this.rows.map(r=>r.si), es=this.rows.filter(r=>r.E!=null).map(r=>r.E), spm=this.rows.filter(r=>r.spm!=null).map(r=>r.spm), off=this.rows.filter(r=>r.off!=null).map(r=>r.off);
    const B=this.meta.binSpm, bins={}; for(const r of this.rows){ if(r.spm==null) continue; const k=String(Math.floor(r.spm/B)*B); (bins[k]=bins[k]||[]).push(r.si); }
    const binOut={}; for(const k of Object.keys(bins)) binOut[k]={n:bins[k].length, mu:mean(bins[k]), sigma:stdOf(bins[k])};
    return Object.assign({}, this.meta, { date:(dateMs==null? null : new Date(dateMs).toISOString()), n:si.length, mu:(si.length? mean(si) : null), sigma:(si.length? stdOf(si) : null),
      eMu:(es.length? mean(es) : null), eSigma:(es.length? stdOf(es) : null), spmMean:(spm.length? mean(spm) : null), offMu:(off.length? mean(off) : null), bins:binOut }); }
}
// 기준선 유효성(가져오기/로드): schema 1, n/mu/sigma 숫자, (선택) 지표·창 길이 일치
function validateBaseline(o, expect){ if(!o || typeof o!=='object') return {ok:false, why:'객체 아님'};
  if(o.schema!==1) return {ok:false, why:'schema '+o.schema+' 미지원'};
  if(!(o.n>=1) || typeof o.mu!=='number' || typeof o.sigma!=='number' || !Number.isFinite(o.mu) || !Number.isFinite(o.sigma)) return {ok:false, why:'n/mu/sigma 없음'};
  if(expect && expect.metric && o.metric!==expect.metric) return {ok:false, why:'지표 불일치('+o.metric+'≠'+expect.metric+')'};
  if(expect && expect.winSec && o.winSec!==expect.winSec) return {ok:false, why:'창 길이 불일치('+o.winSec+'s≠'+expect.winSec+'s)'};
  return {ok:true}; }
// 편차: 케이던스 구간(bin) n≥4 이면 구간 μ/σ, 아니면 전체. thr=max(kSigma·σ, floorPct). 반환 {mu,sigma,thr,delta,z,exceed,src}|null
function baselineDeviation(base, R, cfg){ const c=Object.assign({}, GATE.baseline, cfg||{}); if(!base || base.mu==null || !R || typeof R.signedSI!=='number') return null;
  let mu=base.mu, sigma=base.sigma, src='all'; const B=base.binSpm||c.binSpm;
  if(R.stepSpm!=null && base.bins){ const k=String(Math.floor(R.stepSpm/B)*B), b=base.bins[k]; if(b && b.n>=4){ mu=b.mu; sigma=b.sigma; src='bin'+k; } }
  const thr=Math.max(c.kSigma*(sigma||0), c.floorPct), delta=R.signedSI-mu, z=(sigma>1e-9)? delta/sigma : null;
  return {mu, sigma, thr, delta, z, exceed:Math.abs(delta)>thr, src}; }
// 절대 임계 폴백(기준선 없음): μ=0, |SI|≥thrPct
function absoluteDeviation(R, thrPct){ if(!R || typeof R.signedSI!=='number') return null; return {mu:0, sigma:null, thr:thrPct, delta:R.signedSI, z:null, exceed:Math.abs(R.signedSI)>=thrPct, src:'abs'}; }
function baselineIsStale(base, nowMs, staleDays){ if(!base || !base.date) return true; const d=(nowMs-Date.parse(base.date))/86400000; return !(d>=0 && d<=(staleDays==null? GATE.baseline.staleDays : staleDays)); }
// 판정기: ok 창의 편차가 같은 방향으로 consecutive 창 연속 초과 → intervene. hold/info 는 카운터 유지, not_running 은 리셋, 마지막 ok 창 이후 expireMs 경과 시 리셋.
// mismatch: 세션 첫 offsetWindows 개 ok 창이 모두 같은 방향으로 초과(시작부터 일관 이탈 → 마운트/기준선 확인 권고; 개입 규칙은 그대로)
class Decider {
  constructor(cfg){ this.cfg=Object.assign({}, GATE.baseline, cfg||{}); this.reset(); }
  reset(){ this.count=0; this.sign=0; this.lastOkT=null; this.first=[]; this.mismatch=false; }
  feed(R, dev, tMs, expireMs){
    if(!R) return {action:'none', count:this.count, mismatch:this.mismatch};
    if(R.state==='not_running'){ this.count=0; this.sign=0; return {action:'none', count:0, why:'reset_not_running', mismatch:this.mismatch}; }
    if(R.state!=='ok' || !dev) return {action:'none', count:this.count, why:'keep', mismatch:this.mismatch};
    if(this.lastOkT!=null && expireMs!=null && tMs-this.lastOkT>expireMs){ this.count=0; this.sign=0; }
    this.lastOkT=tMs;
    const s=(dev.delta<0)? -1 : 1;
    if(this.first.length<this.cfg.offsetWindows){ this.first.push(dev.exceed? s : 0);
      if(this.first.length===this.cfg.offsetWindows && this.first.every(v=>v!==0 && v===this.first[0])) this.mismatch=true; }
    if(dev.exceed){ if(s===this.sign) this.count++; else { this.sign=s; this.count=1; } } else { this.count=0; this.sign=0; }
    if(this.count>=this.cfg.consecutive){ this.count=0; this.sign=0; return {action:'intervene', count:this.cfg.consecutive, delta:dev.delta, sign:s, mismatch:this.mismatch}; }
    return {action:'none', count:this.count, delta:dev.delta, sign:s, mismatch:this.mismatch};
  }
}
// 저장소 어댑터(localStorage 등 getItem/setItem/removeItem). 모든 접근 try/catch — 사생활 모드·차단·용량초과에도 예외 없음
class BaselineStore {
  constructor(storage){ this.storage=storage||null; }
  key(deviceId, metric){ return 'mc.baseline.v1.'+(deviceId||'nodev')+'.'+metric; }
  load(deviceId, metric, expect){ try{ if(!this.storage) return null; const s=this.storage.getItem(this.key(deviceId,metric)); if(!s) return null; const o=JSON.parse(s); return validateBaseline(o, expect).ok? o : null; } catch(e){ return null; } }
  save(deviceId, metric, obj){ try{ if(!this.storage) return false; this.storage.setItem(this.key(deviceId,metric), JSON.stringify(obj)); return true; } catch(e){ return false; } }
  clear(deviceId, metric){ try{ if(!this.storage) return false; this.storage.removeItem(this.key(deviceId,metric)); return true; } catch(e){ return false; } }
}

// 전환기 호환 별칭: ok / cadence / harmE / asymPct (앱 코드가 상태 계약으로 옮겨가면 제거)
function legacyAliases(R){ R.ok=(R.state==='ok'); R.cadence=R.strideHz; R.harmE=(R.E==null?0:R.E); R.asymPct=(R.si==null?0:R.si); return R; }

const MC={ ENGINE_VER, GATE, gateWith, REASONS, vperp, leanAbout, gravityUp, yawRate, angleDeg, mean, stdOf, median, detrend, acfPeriod, effectivePeriod,
  goertzelHann, harmonicRatio, turnGate, rampGate, lobes, lobeAlternationT, resolveStride, pairLobes, trimmedSI, zeroShiftFromOutliers, peaksOf,
  reconstructTimes, gapStats, resample, sanityFilter, referenceJump, gridOf, RawEngine, AsymEngine,
  BaselineStats, validateBaseline, baselineDeviation, absoluteDeviation, baselineIsStale, Decider, BaselineStore };
root.MC=MC;
if(typeof module!=='undefined' && module.exports) module.exports=MC;
})(typeof window!=='undefined'?window:globalThis);
