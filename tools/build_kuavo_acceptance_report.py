#!/usr/bin/env python3
"""Build a local acceptance index from recorded outcomes, never inferred success."""
import argparse,html,json,os,hashlib
from pathlib import Path


def read(path,default=None):return json.loads(path.read_text()) if path.is_file() else default


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path('outputs/kuavo_sim/acceptance_20261014'))
    p.add_argument('--manifest',type=Path);a=p.parse_args();a.root=a.root.resolve()
    manifest=read(a.manifest or a.root/'cases.json',{'cases':[]});rows=[];cards=[]
    def link(path):return os.path.relpath(Path(path).resolve(),a.root)
    for case in manifest['cases']:
        state=Path(case['state_dir']).resolve();scene=Path(case['scene']).resolve();folder=Path(case.get('output_dir',scene.parent)).resolve()
        result=read(state/'closed_loop_result.json',read(state/'closed_loop_error.json',{}))
        entries=result.get('results',[]);completed=[r['bag'] for r in entries if r['result'].get('status')=='executed']
        status=result.get('status','pending');weights={r['bag']:[w.get('display_grams') for w in r['result'].get('weights',[])] for r in entries}
        audit=read(Path(case.get('audit',folder/'trajectory_audit.json')),{})
        physical=read(Path(case.get('physical',folder/'physical_score.json')),{})
        fixtures=read(folder/'fixture_check.json',{})
        drops=[read(f) for f in (state/'private_bin_drops').glob('*.json')]
        contact_ok=all(d.get('private_contact_score_passed',False) for d in drops) if drops else None
        acceptance=read(folder/'acceptance_audit.json',{})
        continuity=read(folder/'state_history_check.json',{})
        rgb_weights=read(folder/'rgb_weight_check.json',{})
        verified=status=='executed' and len(completed)==len(case.get('bags',['white','green','black'])) and acceptance.get('passed',False) and continuity.get('passed',False) and rgb_weights.get('passed',False) and audit.get('passed',False) and physical.get('passed_necessary_checks',False) and fixtures.get('source_fixture_passed',False) and contact_ok is True
        video=Path(case.get('video',folder/'replay.mp4'));metadata=read(video.with_suffix('.json'),{})
        # An older movie at the same path must not be presented as the new run.
        history=state/'history.jsonl';video_current=False
        if video.is_file() and metadata and history.is_file():
            records=[json.loads(line) for line in history.read_text().splitlines()]
            records=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed']
            fps=metadata.get('fps',0)
            speed=metadata.get('playback_speed',1)
            expected=sum(max(2,round(r.get('actual_duration_s',2)*fps/speed)) for r in records)
            video_current=metadata.get('actions')==len(records) and metadata.get('frames')==expected and metadata.get('scene')==str(scene)
            if metadata.get('state_dir'):video_current=video_current and metadata['state_dir']==str(state)
            if metadata.get('history_sha256'):video_current=video_current and metadata['history_sha256']==hashlib.sha256(history.read_bytes()).hexdigest()
        if video.is_file() and metadata.get('scope','').startswith('failed input snapshots'):
            video_current=True
        row={**case,'status':status,'completed_bags':completed,'weights_grams':weights,
            'trajectory_audit_passed':audit.get('passed'),'physical_necessary_checks_passed':physical.get('passed_necessary_checks'),
            'source_fixture_passed':fixtures.get('source_fixture_passed'),'state_continuity_passed':continuity.get('passed'),'rgb_weights_verified':rgb_weights.get('passed'),'drop_contact_score_passed':contact_ok,'verified_scoped_simulation':verified,'video_exists':video.is_file(),'video_matches_current_run':video_current}
        rows.append(row)
        reason=result.get('reason','') or next((r['result'].get('reason','') for r in reversed(entries) if r['result'].get('status')!='executed'),'')
        if verified:badge='已通过所列仿真检查'
        elif acceptance.get('passed') is False:badge='动作记录完整，独立检查未通过'
        elif status=='executed':badge='动作已完成，检查待齐'
        else:badge=status
        if acceptance.get('passed') is False:
            failed=[item['check'] for item in acceptance.get('checks',[]) if item['exit_code']!=0]
            reason='独立检查未通过：'+', '.join(failed)+'. '+reason
            hits=physical.get('rigid_collisions',[])
            if hits:
                first=hits[0]
                reason+=f' 首处：{first.get("stage")}，{first.get("robot_body")} 与 {first.get("obstacle")} 穿透 {first.get("penetration_mm",0):.2f} mm。'
        movie=f'<video controls preload="metadata" src="{html.escape(link(video))}"></video><p><button onclick="this.parentNode.previousElementSibling.playbackRate=1">1×</button> <button onclick="this.parentNode.previousElementSibling.playbackRate=2">2×</button> <a href="{html.escape(link(video))}">打开视频</a></p>' if video_current else '<p>本次运行的完整录像尚未生成。</p>'
        if case.get('replay_scope'):movie+='<p>录像范围：'+html.escape(case['replay_scope'])+'</p>'
        commands=f'bash tools/kuavo_tabletop.sh gui --scene {scene} --state-dir {state} --replay'
        evidence=[('场景 XML',scene),('场景参数',scene.parent/'scenario.json'),('执行记录',state/'closed_loop_result.json'),
                  ('异常记录',state/'closed_loop_error.json'),('物理审查',folder/'physical_score.json'),
                  ('初始布置检查',folder/'fixture_check.json'),('预检失败',state/'task_preflight.json')]
        evidence_links=' · '.join(f'<a href="{html.escape(link(path))}">{title}</a>' for title,path in evidence if path.is_file())
        cards.append(f'<article><h2>{html.escape(case["label"])}</h2><p class="badge">{html.escape(badge)}</p><p>{html.escape(case.get("conditions",""))}</p>{movie}<p>完成：{html.escape(", ".join(completed) or "无")}；读数（g）：{html.escape(str(weights))}</p><p>轨迹：{audit.get("passed","未产生")}；物理必要条件：{physical.get("passed_necessary_checks","未产生")}；释放接触：{contact_ok}</p><p>{html.escape(reason[:500])}</p><p>{evidence_links}</p><details><summary>GUI 回放命令</summary><pre>{html.escape(commands)}</pre></details></article>')
    physics=a.root/'deep_gap30/physics_no_grasp_rotor.mp4';facts=read(physics.with_suffix('.json'),{})
    counter=f'<article><h2>实际动力学对照：没有抓手驱动</h2><video controls preload="metadata" src="{link(physics)}"></video><p>浮动机器人、真实电机力矩、自由刚体包裹；没有指爪驱动或附着约束。</p><p>TCP 最大跟踪误差 {facts.get("max_TCP_tracking_error_mm","?")} mm；掌法向偏差 {facts.get("max_palm_normal_error_deg","?")}°；包裹实际抬升 {facts.get("max_actual_parcel_lift_mm","?")} mm。</p><a href="{link(physics.with_suffix(".json"))}">完整物理记录</a></article>'
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Kuavo-5 多条件仿真验收</title><style>body{font:16px/1.6 system-ui,sans-serif;margin:0;background:#edf0f4;color:#18212d}main{max-width:1280px;margin:auto;padding:28px}article{background:white;border-radius:12px;padding:22px;margin:20px 0;border:1px solid #ccd4df}video{width:100%;max-height:720px;background:#151515}pre{white-space:pre-wrap;word-break:break-all;background:#f3f5f8;padding:14px}.badge{font-weight:bold;color:#125487}a{color:#1260a8}button{padding:5px 16px}</style><main><h1>Kuavo-5 多条件仿真验收</h1><p>主流程以辅助抓取检验 RGB 定位、四点坐标转换和 14 维运动规划，释放使用接触沉降。视频右下角是实际输入快照。该结果不能证明真实手指抓取、软袋、颗粒或真机全身控制。</p><p>四点输入目前是仿真假设值；人工确认工具已实现，没有伪造现场人工触达记录。所有场景采用 Kuavo-5.2 biped_s52。</p><p><a href="cases_summary.json">机器可读汇总</a> · <a href="../../../docs/kuavo5_rgb_planar_weighing.md">人工示教与运行文档</a></p>'''
    verified_count=sum(r['verified_scoped_simulation'] for r in rows)
    summary=f'<article><h2>本轮验收汇总</h2><p>{len(rows)} 组场景录像；{verified_count} 组完整三件流程通过所列仿真检查；另有实际动力学对照录像。</p><p>失败、超时和部分流程也保留录像与原因，不能算作成功。完整通过录像为每组63段辅助动作，通常按2倍仿真时间播放；各片实际倍速见标题及元数据。</p><p><a href="delivery_artifact_check.json">全部录像逐帧解码和证据检查</a> · <a href="completion_audit.json">逐项交付审查</a> · <a href="vision_validation_summary.json">40组独立视觉测试</a></p></article>'
    page+=summary+''.join(cards)+counter+'</main></html>'
    (a.root/'index.html').write_text(page);(a.root/'cases_summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    print({'cases':len(rows),'verified_scoped':sum(r['verified_scoped_simulation'] for r in rows),'videos':sum(r['video_exists'] for r in rows)})


if __name__=='__main__':main()
