// 「选择数据目录」对话框
// 通过后端 /browse_dir 浏览服务器文件系统，选定后用 /set_data_root 切换
// 当前浏览器会话的数据根（scene 的父目录），随后整页重载以加载新目录的数据。

export class DirPicker {
    constructor(onPicked) {
        this.onPicked = onPicked;
        this.path = "";
        this.root = null;   // 当前后端生效的数据根
        this.ui = null;
        this._build();
    }

    _el(tag, props, children) {
        let el = document.createElement(tag);
        if (props) {
            for (let k in props) {
                if (k === "style") el.style.cssText = props[k];
                else if (k === "text") el.textContent = props[k];
                else if (k.startsWith("on")) el[k] = props[k];
                else el.setAttribute(k, props[k]);
            }
        }
        (children || []).forEach(c => el.appendChild(c));
        return el;
    }

    _build() {
        let self = this;
        let row = (children) => this._el("div", {style: "display:flex;align-items:center;gap:8px;"}, children);
        let btn = (text, onclick, style) => this._el("button", {
            text: text, onclick: onclick,
            style: "cursor:pointer;padding:5px 12px;border-radius:4px;border:1px solid #666;" +
                "background:#3c3f41;color:#e6e6e6;font-size:13px;" + (style || ""),
        });

        this.pathInput = this._el("input", {
            type: "text",
            style: "flex:1;min-width:280px;padding:5px 8px;border-radius:4px;border:1px solid #666;" +
                "background:#2b2b2b;color:#e6e6e6;font-size:13px;",
        });
        this.pathInput.addEventListener("keydown", (e) => {
            if (e.key === "Enter") { e.preventDefault(); self.load(self.pathInput.value); }
        });

        this.info = this._el("div", {
            style: "font-size:12px;color:#9fb3c8;min-height:16px;margin:2px 0;",
        });

        this.listUi = this._el("div", {
            style: "flex:1;overflow:auto;border:1px solid #555;border-radius:4px;background:#2b2b2b;min-height:220px;",
        });

        this.statusUi = this._el("div", {
            style: "font-size:12px;color:#e0a458;min-height:16px;",
        });

        let panel = this._el("div", {
            style: "width:640px;max-width:92vw;max-height:80vh;display:flex;flex-direction:column;gap:8px;" +
                "background:#383838;border:1px solid #666;border-radius:8px;padding:14px;" +
                "box-shadow:0 12px 40px rgba(0,0,0,.6);color:#e6e6e6;" +
                "font-family:inherit;",
        }, [
            row([
                this._el("span", {text: "选择数据目录", style: "font-size:15px;font-weight:600;flex:1;"}),
                this._el("span", {
                    text: "\u2715", title: "关闭", onclick: () => self.close(),
                    style: "cursor:pointer;padding:2px 8px;font-size:14px;",
                }),
            ]),
            row([
                this.pathInput,
                btn("转到", () => self.load(self.pathInput.value)),
            ]),
            row([
                btn("\u2191 上级目录", () => self.load(self.parent)),
                btn("\u2302 主目录", () => self.load(self.home)),
                btn("\u2317 项目 data", () => self.load(self.defaultRoot)),
                this._el("span", {style: "flex:1;"}),
            ]),
            this.info,
            this.listUi,
            this.statusUi,
            row([
                this._el("span", {style: "flex:1;"}),
                btn("恢复默认目录", () => self.apply("")),
                btn("使用当前目录", () => self.apply(self.path),
                    "background:#2f6f3b;border-color:#4caf50;font-weight:600;"),
            ]),
        ]);

        let overlay = this._el("div", {
            style: "position:fixed;inset:0;z-index:99999;background:rgba(0,0,0,.5);" +
                "display:none;align-items:center;justify-content:center;",
            onclick: (e) => { if (e.target === overlay) self.close(); },
        }, [panel]);

        document.body.appendChild(overlay);
        this.ui = overlay;
    }

    open(startPath) {
        this.ui.style.display = "flex";
        this.load(startPath || this.path || this.root || this.defaultRoot);
    }

    close() {
        this.ui.style.display = "none";
    }

    _url(endpoint, params) {
        let qs = Object.keys(params).map(k => k + "=" + encodeURIComponent(params[k])).join("&");
        return "/" + endpoint + (qs ? "?" + qs : "");
    }

    load(path) {
        let self = this;
        this.statusUi.textContent = "读取中…";
        fetch(this._url("browse_dir", {path: path == null ? "" : path}))
            .then(r => r.json())
            .then(ret => {
                if (!ret.ok) {
                    self.statusUi.textContent = ret.error || "读取失败";
                    return;
                }
                self.path = ret.path;
                self.parent = ret.parent;
                self.home = ret.home;
                self.defaultRoot = ret.default_root;
                self.statusUi.textContent = "";
                self.pathInput.value = ret.path;
                self._render(ret);
            })
            .catch(err => { self.statusUi.textContent = "读取失败: " + err; });
    }

    _render(ret) {
        let self = this;
        let frag = document.createDocumentFragment();

        // 当前目录自身的提示
        let sceneNames = ret.scene_names || [];
        if (ret.is_scene) {
            this.info.textContent = "当前目录是一个 scene（" + ret.path.split("/").pop() +
                "），使用后会打开它的父目录并载入该 scene。";
        } else if (sceneNames.length > 0) {
            this.info.textContent = "当前目录下有 " + sceneNames.length + " 个 scene，可直接使用。";
        } else {
            this.info.textContent = "当前目录下没有识别到 scene（需要包含 scene 子目录）。";
        }

        let makeItem = (label, path, isScene, enterOnly) => {
            let name = self._el("span", {text: label, style: "flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;"});
            let badge = isScene
                ? self._el("span", {text: "scene",
                    style: "font-size:11px;color:#8bc34a;border:1px solid #4caf50;border-radius:3px;padding:0 5px;margin-right:6px;"})
                : null;
            let right = self._el("div", {style: "display:flex;align-items:center;"});
            if (badge) right.appendChild(badge);
            let pick = self._el("span", {
                text: "选中", title: "使用此目录",
                style: "font-size:12px;color:#8bc34a;cursor:pointer;padding:2px 8px;border:1px solid #4caf50;border-radius:3px;margin-right:6px;",
                onclick: (e) => { e.stopPropagation(); self.apply(path); },
            });
            right.appendChild(pick);
            let enter = self._el("span", {text: "\u203A", style: "width:14px;text-align:center;color:#9fb3c8;"});

            let item = self._el("div", {
                style: "display:flex;align-items:center;padding:5px 10px;cursor:pointer;border-bottom:1px solid #333;",
                onclick: () => self.load(path),
            }, [name, right, enter]);

            if (enterOnly) { pick.style.display = "none"; }
            return item;
        };

        // 上级目录
        if (ret.parent) {
            frag.appendChild(this._el("div", {
                style: "display:flex;align-items:center;padding:5px 10px;cursor:pointer;border-bottom:1px solid #333;color:#9fb3c8;",
                text: "\u2191 ..",
                onclick: () => self.load(ret.parent),
            }));
        }

        if (!ret.dirs || ret.dirs.length === 0) {
            frag.appendChild(this._el("div", {
                text: "（没有子目录）",
                style: "padding:10px;color:#888;font-size:13px;",
            }));
        } else {
            ret.dirs.forEach(d => {
                frag.appendChild(makeItem(d.name, d.path, d.is_scene, false));
            });
        }

        this.listUi.innerHTML = "";
        this.listUi.appendChild(frag);
    }

    apply(path) {
        let self = this;
        this.statusUi.textContent = "切换中…";
        fetch("/set_data_root", {
            method: "POST",
            headers: {"Content-Type": "application/x-www-form-urlencoded"},
            body: "path=" + encodeURIComponent(path == null ? "" : path),
        })
            .then(r => r.json())
            .then(ret => {
                if (!ret.ok) {
                    self.statusUi.textContent = ret.error || "切换失败";
                    return;
                }
                self.root = ret.root;
                self.close();
                if (self.onPicked) self.onPicked(ret);
            })
            .catch(err => { self.statusUi.textContent = "切换失败: " + err; });
    }
}
