#!/usr/bin/env bash
# ===========================================================================
# Full RoboLLM development-environment setup for Ubuntu 24.04 (noble)
#
#   RUN AS YOUR NORMAL USER (NOT sudo):   bash ~/Desktop/dev-setup.sh
#
# Installs: VS Code (+extensions), Docker, gcloud, AWS CLI v2,
#   Python (pipx/uv/pyenv), zsh+oh-my-zsh+atuin, Jujutsu (jj),
#   database tools (DBeaver + CLI clients), dev CLI tools,
#   FreeCAD 1.0 + CROSS workbench + FreeCAD-MCP (registered with Claude Code).
#
# Safe to re-run: every step is idempotent. It first repairs any interrupted
# apt/dpkg, and every apt call waits up to 10 min for the lock instead of
# failing (fixes the unattended-upgrades race).
# ===========================================================================
set -uo pipefail

log()  { printf '\n\033[1;34m==>\033[0m \033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[1;32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;31m✗\033[0m %s\n' "$*"; }
skip() { printf '  \033[1;33m•\033[0m %s (already present)\n' "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }

# apt helpers: always wait for the lock, never fail on a busy dpkg
APTLOCK="-o DPkg::Lock::Timeout=600"
apt_update()  { sudo apt-get $APTLOCK update -qq; }
apt_install() { sudo apt-get $APTLOCK install -y -qq "$@"; }

# install a single-binary tool from a jesseduffield-style GitHub release
# (resolves the version via the latest-release redirect — robust to API rate limits)
gh_bin() { # $1 = owner/repo   $2 = binary name
  local repo="$1" bin="$2" ver url tmp
  have "$bin" && { skip "$bin"; return; }
  ver=$(curl -fsSLI "https://github.com/$repo/releases/latest" | grep -i '^location:' | grep -oE 'v[0-9.]+' | tr -d v | head -1)
  [[ -z "$ver" ]] && { warn "$bin: could not resolve latest version"; return; }
  url="https://github.com/$repo/releases/download/v${ver}/${bin}_${ver}_Linux_$(uname -m).tar.gz"
  tmp=$(mktemp -d)
  if curl -fsSL "$url" -o "$tmp/x.tar.gz" && tar -xzf "$tmp/x.tar.gz" -C "$tmp" && install -m755 "$tmp/$bin" "$HOME/.local/bin/$bin"; then
    ok "$bin $ver installed"
  else warn "$bin install failed"; fi
  rm -rf "$tmp"
}

if [[ $EUID -eq 0 ]]; then
  echo "Do NOT run with sudo. Run:  bash ~/Desktop/dev-setup.sh" >&2; exit 1
fi

log "Caching sudo credentials (password asked once)"
sudo -v || { echo "sudo required"; exit 1; }
while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done 2>/dev/null &
trap 'kill %1 2>/dev/null' EXIT

# ---------------------------------------------------------------------------
log "0/12  Repair any interrupted / suspended apt or dpkg"
# Kill only *stopped* (suspended, state T) apt/dpkg that would hold the lock forever
for pid in $(pgrep -x apt-get 2>/dev/null) $(pgrep -x dpkg 2>/dev/null) $(pgrep -x apt 2>/dev/null); do
  state=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)
  if [[ "$state" == "T" ]]; then warn "clearing suspended apt/dpkg PID $pid"; sudo kill -9 "$pid" 2>/dev/null; fi
done
sudo dpkg --configure -a 2>/dev/null && ok "dpkg state consistent"
sudo apt-get $APTLOCK -f install -y -qq 2>/dev/null && ok "broken dependencies resolved"

# ---------------------------------------------------------------------------
log "1/12  Base packages & prerequisites"
apt_update
apt_install ca-certificates curl wget gnupg lsb-release apt-transport-https \
  software-properties-common build-essential git zsh unzip \
  make libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev \
  libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev \
  && ok "prerequisites installed" || warn "some prerequisites failed"

# ---------------------------------------------------------------------------
log "2/12  VS Code"
if have code; then skip "VS Code"; else
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
    | gpg --dearmor | sudo tee /usr/share/keyrings/microsoft.gpg >/dev/null
  echo "deb [arch=amd64,arm64,armhf signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
    | sudo tee /etc/apt/sources.list.d/vscode.list >/dev/null
  apt_update
  apt_install code && ok "VS Code installed" || warn "VS Code failed"
fi
if have code; then
  log "     VS Code extensions"
  for ext in \
    ms-iot.vscode-ros ms-python.python ms-python.vscode-pylance \
    ms-vscode.cpptools-extension-pack twxs.cmake ms-vscode.cmake-tools \
    ms-azuretools.vscode-docker ms-vscode-remote.remote-containers \
    ms-vscode-remote.remote-ssh redhat.vscode-yaml \
    amazonwebservices.aws-toolkit-vscode googlecloudtools.cloudcode \
    cweijan.vscode-database-client2 mtxr.sqltools eamodio.gitlens; do
    code --install-extension "$ext" --force >/dev/null 2>&1 && ok "$ext" || warn "ext $ext"
  done
fi

# ---------------------------------------------------------------------------
log "3/12  Docker Engine + Compose plugin"
if have docker; then skip "Docker"; else
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  apt_update
  apt_install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin \
    && ok "Docker installed" || warn "Docker failed"
fi
if id -nG "$USER" | grep -qw docker; then skip "user in docker group"; else
  sudo groupadd -f docker && sudo usermod -aG docker "$USER" \
    && ok "added $USER to docker group (run: newgrp docker)"
fi

# ---------------------------------------------------------------------------
log "4/12  Google Cloud CLI"
if have gcloud; then skip "gcloud"; else
  curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg
  echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" \
    | sudo tee /etc/apt/sources.list.d/google-cloud-sdk.list >/dev/null
  apt_update
  apt_install google-cloud-cli && ok "gcloud installed (run: gcloud init)" || warn "gcloud failed"
fi

# ---------------------------------------------------------------------------
log "5/12  AWS CLI v2"
if have aws; then skip "aws"; else
  tmp=$(mktemp -d)
  curl -fsSL "https://awscli.amazonaws.com/awscli-exe-linux-$(uname -m).zip" -o "$tmp/aws.zip" \
    && unzip -q "$tmp/aws.zip" -d "$tmp" && sudo "$tmp/aws/install" --update \
    && ok "AWS CLI v2 installed (run: aws configure)" || warn "aws failed"
  rm -rf "$tmp"
fi

# ---------------------------------------------------------------------------
log "6/12  Python tooling: pipx, uv, pyenv"
apt_install pipx && pipx ensurepath >/dev/null 2>&1 && ok "pipx ready" || warn "pipx failed"
if have uv || [[ -x "$HOME/.local/bin/uv" ]]; then skip "uv"; else
  curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null 2>&1 && ok "uv installed" || warn "uv failed"
fi
if [[ -d "$HOME/.pyenv" ]]; then skip "pyenv"; else
  curl -fsSL https://pyenv.run | bash >/dev/null 2>&1 && ok "pyenv installed" || warn "pyenv failed"
fi

# ---------------------------------------------------------------------------
log "7/12  zsh + oh-my-zsh + plugins + atuin"
export ZSH="$HOME/.oh-my-zsh"
if [[ -d "$ZSH" ]]; then skip "oh-my-zsh"; else
  RUNZSH=no CHSH=no KEEP_ZSHRC=yes \
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && ok "oh-my-zsh installed" || warn "oh-my-zsh failed"
fi
ZC="${ZSH_CUSTOM:-$ZSH/custom}"
[[ -d "$ZC/plugins/zsh-autosuggestions" ]] || git clone -q https://github.com/zsh-users/zsh-autosuggestions "$ZC/plugins/zsh-autosuggestions"
[[ -d "$ZC/plugins/zsh-syntax-highlighting" ]] || git clone -q https://github.com/zsh-users/zsh-syntax-highlighting "$ZC/plugins/zsh-syntax-highlighting"
ok "zsh plugins present"
if have atuin || [[ -x "$HOME/.atuin/bin/atuin" ]]; then skip "atuin"; else
  curl -fsSL https://setup.atuin.sh | sh >/dev/null 2>&1 && ok "atuin installed" || warn "atuin failed"
fi
# atuin installs to ~/.atuin/bin (not on PATH for sh/scripts) — link it into ~/.local/bin
[[ -x "$HOME/.atuin/bin/atuin" ]] && ln -sf "$HOME/.atuin/bin/atuin" "$HOME/.local/bin/atuin"

# ---------------------------------------------------------------------------
log "8/12  Writing ~/.zshrc"
ZRC="$HOME/.zshrc"
[[ -f "$ZRC" ]] && cp "$ZRC" "$ZRC.bak.$(date +%s 2>/dev/null || echo backup)" 2>/dev/null
cat > "$ZRC" <<'ZSHRC'
# ~/.zshrc — generated by dev-setup.sh
export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME="robbyrussell"
plugins=(
  git gh docker docker-compose gcloud aws python pip direnv fzf
  colored-man-pages command-not-found
  zsh-autosuggestions zsh-syntax-highlighting   # syntax-highlighting last
)
source "$ZSH/oh-my-zsh.sh"

export PATH="$HOME/.local/bin:$PATH"

export PYENV_ROOT="$HOME/.pyenv"
[[ -d "$PYENV_ROOT/bin" ]] && export PATH="$PYENV_ROOT/bin:$PATH"
command -v pyenv >/dev/null && eval "$(pyenv init - zsh)"

command -v uv >/dev/null && eval "$(uv generate-shell-completion zsh)"

autoload -Uz bashcompinit && bashcompinit
autoload -Uz compinit && compinit

# ROS 2 (Jazzy)
if [[ -f /opt/ros/jazzy/setup.zsh ]]; then
  source /opt/ros/jazzy/setup.zsh
  if command -v register-python-argcomplete3 >/dev/null; then
    eval "$(register-python-argcomplete3 ros2)" 2>/dev/null
    eval "$(register-python-argcomplete3 colcon)" 2>/dev/null
  fi
  [[ -f "$HOME/ros2_ws/install/setup.zsh" ]] && source "$HOME/ros2_ws/install/setup.zsh"
fi

[[ -f /usr/share/google-cloud-sdk/completion.zsh.inc ]] && source /usr/share/google-cloud-sdk/completion.zsh.inc
command -v direnv >/dev/null && eval "$(direnv hook zsh)"
command -v atuin  >/dev/null && eval "$(atuin init zsh)"

alias ll='ls -alh'
alias dc='docker compose'
alias k='kubectl'
alias lg='lazygit'
alias lzd='lazydocker'
command -v batcat >/dev/null && alias bat='batcat' && alias cat='batcat -pp'
command -v fdfind >/dev/null && alias fd='fdfind'
ZSHRC
ok "~/.zshrc written (old one backed up)"
if [[ "$SHELL" == *zsh ]]; then skip "default shell already zsh"; else
  sudo chsh -s "$(command -v zsh)" "$USER" && ok "default shell -> zsh (next login)"
fi

# ---------------------------------------------------------------------------
log "9/12  Jujutsu (jj)"
mkdir -p "$HOME/.local/bin"
if have jj || [[ -x "$HOME/.local/bin/jj" ]]; then skip "jujutsu"; else
  arch=$(uname -m)
  url=$(curl -fsSL https://api.github.com/repos/jj-vcs/jj/releases/latest \
    | grep -oE '"browser_download_url": *"[^"]*'"${arch}"'-unknown-linux-musl.tar.gz"' | head -1 | cut -d'"' -f4)
  if [[ -n "$url" ]]; then
    tmp=$(mktemp -d); curl -fsSL "$url" -o "$tmp/jj.tar.gz" && tar -xzf "$tmp/jj.tar.gz" -C "$tmp"
    install -m755 "$(find "$tmp" -name jj -type f | head -1)" "$HOME/.local/bin/jj" && ok "jj installed" || warn "jj failed"
    rm -rf "$tmp"
  else warn "could not resolve jj release URL"; fi
fi
JJ="$(command -v jj || echo "$HOME/.local/bin/jj")"
if [[ -x "$JJ" ]]; then
  gn=$(git config --global user.name 2>/dev/null); ge=$(git config --global user.email 2>/dev/null)
  [[ -n "$gn" ]] && "$JJ" config set --user user.name  "$gn" 2>/dev/null
  [[ -n "$ge" ]] && "$JJ" config set --user user.email "$ge" 2>/dev/null
fi

# ---------------------------------------------------------------------------
log "10/12  Database tools"
apt_install postgresql-client default-mysql-client sqlite3 redis-tools \
  && ok "psql / mysql / sqlite3 / redis-cli" || warn "DB CLI clients failed"
for t in pgcli mycli litecli; do
  pipx install "$t" >/dev/null 2>&1 && ok "$t" || skip "$t"
done
if have dbeaver || dpkg -l 2>/dev/null | grep -q dbeaver-ce; then skip "DBeaver"; else
  wget -qO- https://dbeaver.io/debs/dbeaver.gpg.key | sudo gpg --dearmor -o /usr/share/keyrings/dbeaver.gpg
  echo "deb [signed-by=/usr/share/keyrings/dbeaver.gpg] https://dbeaver.io/debs/dbeaver-ce /" \
    | sudo tee /etc/apt/sources.list.d/dbeaver.list >/dev/null
  apt_update && apt_install dbeaver-ce && ok "DBeaver installed" || warn "DBeaver failed"
fi

# ---------------------------------------------------------------------------
log "11/12  Developer CLI tools"
apt_install ripgrep fd-find bat jq tree ncdu htop btop tmux tldr httpie direnv fzf \
  && ok "ripgrep, fd, bat, jq, tree, ncdu, htop, btop, tmux, tldr, httpie, direnv, fzf" \
  || warn "some CLI tools failed"
if have gh; then skip "gh"; else
  curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg
  sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    | sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
  apt_update && apt_install gh && ok "gh installed" || warn "gh failed"
fi
# lazygit (git TUI) and lazydocker (docker TUI — lightweight Docker Desktop replacement)
gh_bin jesseduffield/lazygit    lazygit
gh_bin jesseduffield/lazydocker lazydocker

# ---------------------------------------------------------------------------
log "12/12  FreeCAD 1.0 + CROSS workbench + FreeCAD-MCP"
if have freecad; then skip "FreeCAD"; else
  sudo add-apt-repository -y ppa:freecad-maintainers/freecad-stable >/dev/null 2>&1
  apt_update
  apt_install freecad && ok "FreeCAD installed" || warn "FreeCAD failed"
fi
FC_MOD="$HOME/.local/share/FreeCAD/Mod"; mkdir -p "$FC_MOD"
if [[ -d "$FC_MOD/freecad.cross" ]]; then skip "CROSS workbench"; else
  git clone -q https://github.com/galou/freecad.cross "$FC_MOD/freecad.cross" && ok "CROSS workbench" || warn "CROSS failed"
fi
if [[ -d "$FC_MOD/FreeCADMCP" ]]; then skip "FreeCAD-MCP addon"; else
  tmp=$(mktemp -d); git clone -q https://github.com/neka-nat/freecad-mcp "$tmp/fcmcp"
  if [[ -d "$tmp/fcmcp/addon/FreeCADMCP" ]]; then cp -r "$tmp/fcmcp/addon/FreeCADMCP" "$FC_MOD/" && ok "FreeCAD-MCP addon"; else warn "MCP addon path not found"; fi
  rm -rf "$tmp"
fi
export PATH="$HOME/.local/bin:$PATH"
if have claude; then
  if claude mcp list 2>/dev/null | grep -q '^freecad'; then skip "freecad MCP registered"; else
    claude mcp add --scope user freecad -- uvx freecad-mcp >/dev/null 2>&1 \
      && ok "freecad MCP registered (user scope)" \
      || warn "register later: claude mcp add --scope user freecad -- uvx freecad-mcp"
  fi
else warn "Claude CLI not on PATH — register MCP later"; fi

# ===========================================================================
log "VERIFICATION — what actually installed"
fail=0
while IFS='|' read -r cmd label; do
  [[ -z "$cmd" ]] && continue
  if command -v "$cmd" >/dev/null 2>&1 \
     || [[ -x "$HOME/.local/bin/$cmd" || -x "$HOME/.pyenv/bin/$cmd" || -x "$HOME/.atuin/bin/$cmd" ]]; then
    ok "$label ($cmd)"
  else warn "$label ($cmd) MISSING"; fail=$((fail+1)); fi
done <<'LIST'
code|VS Code
docker|Docker
gcloud|Google Cloud CLI
aws|AWS CLI v2
pipx|pipx
uv|uv
pyenv|pyenv
zsh|zsh
atuin|atuin
jj|Jujutsu
freecad|FreeCAD
gh|GitHub CLI
lazygit|lazygit
lazydocker|lazydocker
psql|Postgres client
mysql|MySQL client
sqlite3|SQLite
redis-cli|Redis client
pgcli|pgcli
dbeaver|DBeaver
rg|ripgrep
fdfind|fd
batcat|bat
jq|jq
fzf|fzf
htop|htop
btop|btop
tmux|tmux
direnv|direnv
http|httpie
tldr|tldr
LIST

echo
if [[ $fail -eq 0 ]]; then
  printf '\033[1;32mALL TOOLS INSTALLED ✓\033[0m\n'
else
  printf '\033[1;31m%d tool(s) still missing — re-run this script (it retries only those).\033[0m\n' "$fail"
fi

cat <<'EOF'

Next steps:
  • New shell:        exec zsh
  • Docker no-sudo:   newgrp docker    then   docker run --rm hello-world
  • Clouds:           gcloud init      and    aws configure
  • Editor:           code .
  • FreeCAD MCP:      open FreeCAD -> "MCP Addon" workbench -> Start RPC Server,
                      then restart Claude Code. Ask Claude to drive FreeCAD / make URDFs.
EOF
