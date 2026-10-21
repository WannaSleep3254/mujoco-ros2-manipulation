"""Render escaped HTML/CSV with packaged templates, without ROS or plotting."""
import csv
from html import escape
from importlib.resources import files
import io
from string import Template
from urllib.parse import quote

from . import GENERATOR_VERSION

CLOCKS = {"wall": "wall", "simulation": "sim", "not_applicable": "해당 없음"}
STATUS = {"preliminary": "예비 평가", "validated": "검증 완료", "superseded": "후속 보고서 있음"}
COVERAGE = {"measured": "측정", "partial": "부분 측정", "estimated": "추정", "unmeasured": "미측정", "proposed": "설계안"}


def esc(value):
    return escape(str(value), quote=True)


RESOURCE_LABELS = {"legacy_report": "상세 리포트", "video": "동작 영상"}


def report_resources(data, prefix=""):
    return [{"url": prefix + quote(asset["path"], safe="/"), "label": asset["label"], "kind": asset["role"]}
            for asset in data["assets"] if asset["role"] in RESOURCE_LABELS]


def resource_links(resources):
    links = []
    for resource in resources:
        caption = RESOURCE_LABELS[resource["kind"]]
        if sum(item["kind"] == resource["kind"] for item in resources) > 1:
            caption += ": " + resource["label"]
        links.append(f'<a data-report-resource="{resource["kind"]}" href="{esc(resource["url"])}" '
                     f'title="{esc(resource["label"])}">{esc(caption)}</a>')
    return "".join(links)


def _page(title, subtitle, body, navigation, script=""):
    resources = files(__package__).joinpath("templates")
    template = Template(resources.joinpath("page.html").read_text(encoding="utf-8"))
    return template.substitute(title=esc(title), subtitle=esc(subtitle), body=body,
        navigation=navigation, generator=GENERATOR_VERSION, script=script,
        style=resources.joinpath("style.css").read_text(encoding="utf-8"))


def _list(items):
    return "<ul>" + "".join(f"<li>{esc(item)}</li>" for item in items) + "</ul>"


def metric_rows(data):
    groups = {group["id"]: group for group in data["sample_groups"]}
    for metric in data["metrics"]:
        for item in metric["values"]:
            group = groups[item["group_id"]]
            yield metric, item, group


def metrics_csv(data):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["report_id", "metric_id", "label", "group_id", "value", "unit", "clock",
                     "sample_count", "statistic", "scope", "note"])
    for metric, item, group in metric_rows(data):
        # Text cells are prefixed when needed to avoid spreadsheet formula evaluation.
        row = [data["report_id"], metric["id"], metric["label"], group["id"],
               "" if item["value"] is None else item["value"], metric["unit"], metric["clock"],
               item["sample_count"], item["statistic"], metric["scope"], metric["note"]]
        writer.writerow(["'" + value if isinstance(value, str) and (value.lstrip().startswith(("=", "+", "-", "@")) or value.startswith(("\t", "\r", "\n"))) else value for value in row])
    return stream.getvalue()


def report_html(data):
    source = data["provenance"]
    parts = ['<section id="summary"><h2>평가 범위와 판정</h2>',
             f'<span class="tag">{STATUS[data["status"]]}</span>',
             f'<p>{esc(data["overview"])}</p>', _list(data["findings"]),
             '<h3>해석의 한계</h3>', _list(data["limitations"]), '</section>']
    parts.append('<section id="pipeline"><h2>지령 → 계산 → 동작의 계측 범위</h2><div class="flow">')
    for stage in data["stages"]:
        parts.append(f'<div class="stage"><h3>{esc(stage["label"])}</h3>'
                     f'<span class="tag">{COVERAGE[stage["coverage"]]} / {CLOCKS[stage["clock"]]}</span>'
                     f'<p>{esc(stage["note"])}</p></div>')
    parts.append('</div></section><section id="metrics"><h2>정량 지표</h2>'
                 '<p class="note">n은 해당 조건에서 이 지표를 집계한 실행/반복 수입니다. 관측 자세 표본 수와 구분합니다. '
                 '서로 다른 시계·실행의 시간을 더해 전체 응답시간으로 사용하지 않습니다.</p>'
                 '<div class="controls"><label>조건 <select id="condition"><option value="all">전체</option>')
    for kind in sorted({g["kind"] for g in data["sample_groups"]}):
        parts.append(f'<option value="{kind}">{kind}</option>')
    parts.append('</select></label><a href="metrics.csv" download>지표 CSV</a> · '
                 '<a href="report.json" download>표준 JSON</a> · <button id="print">인쇄 / PDF</button></div>'
                 '<div class="table-wrap"><table><thead><tr><th>지표</th><th>조건</th><th>값</th><th>단위 / 시계</th>'
                 '<th>집계 / n</th><th>범위·설명</th></tr></thead><tbody>')
    for metric, item, group in metric_rows(data):
        number = '미측정' if item["value"] is None else f'{item["value"]:.6g}'
        parts.append(f'<tr data-kind="{group["kind"]}"><td class="metric-name">{esc(metric["label"])}</td>'
            f'<td>{esc(group["label"])}</td><td>{number}</td><td>{metric["unit"]} / {CLOCKS[metric["clock"]]}</td>'
            f'<td>{item["statistic"]} / {item["sample_count"]}</td>'
            f'<td class="metric-note">{esc(metric["scope"])}<br><span class="note">{esc(metric["note"])}</span></td></tr>')
    parts.append('</tbody></table></div></section><section id="samples"><h2>표본과 평가 조건</h2>')
    parts.append('<div class="table-wrap"><table><thead><tr><th>조건</th><th>고유 사례 / 실행 수</th>'
                 '<th>성공 / 실패</th><th>seed·범위</th></tr></thead><tbody>')
    for group in data["sample_groups"]:
        counts = '미집계' if group["successes"] is None else f'{group["successes"]} / {group["failures"]}'
        parts.append(f'<tr><td>{esc(group["label"])}</td><td>{group["unique_cases"]} / {group["observations"]}</td>'
            f'<td>{counts}</td><td>{esc(group["seed_policy"])}<br>{esc(group["description"])}</td></tr>')
    parts.append('</tbody></table></div></section><section id="evidence"><h2>자료와 재현 정보</h2>')
    for asset in data["assets"]:
        url = quote(asset["path"], safe="/")
        if asset["role"] == "figure":
            parts.append(f'<figure><img src="{url}" alt="{esc(asset["label"])}" loading="lazy">'
                         f'<figcaption>{esc(asset["label"])}</figcaption></figure>')
        else:
            parts.append(f'<p><a href="{url}">{esc(asset["label"])}</a> <span class="note">{asset["role"]}</span></p>')
    commit_url = source["repository"].rstrip("/") + "/commit/" + source["commit"]
    parts.append(f'<p>기준 코드: <a href="{esc(commit_url)}"><code>{source["commit"]}</code></a></p>')
    metadata = {"평가 ID": data["report_id"], "작업장면": source["workcell"],
                "모델": ", ".join(source["models"]), "시나리오": source["scenario"],
                "입력 종류": source["input_kind"], **{"환경 / " + k: v for k, v in data["environment"].items()}}
    parts.append('<dl class="dl">' + ''.join(f'<dt>{esc(k)}</dt><dd>{esc(v)}</dd>' for k, v in metadata.items()) + '</dl>')
    if data.get("supersedes"):
        parts.append(f'<p>이전 보고서: <a href="../{data["supersedes"]}/index.html">{data["supersedes"]}</a></p>')
    parts.append('<p><a href="report-manifest.json">공개 파일 SHA-256·크기·생성기 정보</a></p><h3>관련 자료</h3>')
    parts.append('<ul>' + ''.join(f'<li><a href="{esc(r["url"])}">{esc(r["label"])}</a></li>' for r in data["references"]) + '</ul></section>')
    script = """'use strict';document.getElementById('condition').addEventListener('change',e=>{for(const row of document.querySelectorAll('tr[data-kind]'))row.hidden=e.target.value!=='all'&&row.dataset.kind!==e.target.value});document.getElementById('print').addEventListener('click',()=>window.print());window.reportReady=true;"""
    nav = ('<nav><a href="../index.html">리포트 목록</a>'
           + resource_links(report_resources(data))
           + '<a href="#metrics">지표</a><a href="#evidence">근거</a></nav>')
    return _page(data["title"], f'{data["published_date"]} / {source["workcell"]} / {data["report_id"]}', ''.join(parts), nav, script)


def catalog_entry(data):
    return {"report_id": data["report_id"], "title": data["title"], "published_date": data["published_date"],
            "status": data["status"], "commit": data["provenance"]["commit"],
            "workcell": data["provenance"]["workcell"], "models": data["provenance"]["models"],
            "overview": data["overview"], "url": data["report_id"] + "/index.html",
            "resources": report_resources(data, data["report_id"] + "/")}


def catalog_html(entries):
    body = ['<section><h2>평가 이력</h2><p>같은 조건에서 측정한 지표끼리 비교하세요. 예비 평가와 미측정 구간을 함께 표시합니다.</p>'
            '<div class="controls"><label>리포트 검색 <input id="search" type="search" placeholder="모델·평가명·날짜"></label>'
            '<a href="catalog.json">목록 JSON</a></div><p id="count" class="note" aria-live="polite"></p>'
            '<div class="table-wrap"><table class="catalog"><thead><tr><th>날짜 / 상태</th><th>리포트</th><th>모델·기준</th><th>평가 범위</th></tr></thead><tbody>']
    for entry in entries:
        links = (f'<a data-report-link="summary" href="{esc(entry["url"])}">요약 보기</a>'
                 + resource_links(entry.get("resources", [])))
        body.append(f'<tr data-report><td>{entry["published_date"]}<br><span class="tag">{STATUS[entry["status"]]}</span></td>'
            f'<td><a href="{esc(entry["url"])}">{esc(entry["title"])}</a>'
            f'<div class="report-links">{links}</div><code>{entry["report_id"]}</code></td>'
            f'<td>{esc(", ".join(entry["models"]))}<br><code>{entry["commit"][:12]}</code></td><td>{esc(entry["overview"])}</td></tr>')
    body.append('</tbody></table></div></section>')
    script = """'use strict';const rows=[...document.querySelectorAll('[data-report]')];function filter(){const q=document.getElementById('search').value.trim().toLowerCase();let n=0;for(const row of rows){row.hidden=!row.textContent.toLowerCase().includes(q);if(!row.hidden)n++}document.getElementById('count').textContent=n+' / '+rows.length+'개 리포트'}document.getElementById('search').addEventListener('input',filter);filter();window.reportReady=true;"""
    return _page('로봇 제어 성능평가 리포트', '원본 저장소에서 관리하는 평가 이력 · ROS 2 / MuJoCo / MoveIt', ''.join(body), '', script)
