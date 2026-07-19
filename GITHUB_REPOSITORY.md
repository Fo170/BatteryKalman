# GitHub Repository - BatteryKalman

## 📌 Official Repository

**GitHub**: [https://github.com/Fo170/BatteryKalman](https://github.com/Fo170/BatteryKalman)

**Main Branch**: Contains latest stable release (v1.5)

---

## 🔗 Quick Links

| Resource | URL |
|----------|-----|
| **Repository** | https://github.com/Fo170/BatteryKalman |
| **Latest Release** | https://github.com/Fo170/BatteryKalman/releases |
| **Main Branch** | https://github.com/Fo170/BatteryKalman/tree/main |
| **Issues** | https://github.com/Fo170/BatteryKalman/issues |
| **Discussions** | https://github.com/Fo170/BatteryKalman/discussions |
| **Source Code** | https://github.com/Fo170/BatteryKalman/tree/main/src |
| **Examples** | https://github.com/Fo170/BatteryKalman/tree/main/Exemples |

---

## 📥 Clone Latest from Main

```bash
# Clone the repository
git clone https://github.com/Fo170/BatteryKalman.git

# Navigate to directory
cd BatteryKalman

# Ensure on main branch
git checkout main

# Pull latest changes
git pull origin main
```

---

## 📦 Installation Methods

### 1. PlatformIO (Recommended)

```ini
[env:esp32]
lib_deps = 
    Fo170/BatteryModels >= 1.2.0
    Fo170/BatteryKalman >= 1.5
```

### 2. Arduino IDE Library Manager

- Tools → Manage Libraries
- Search: "BatteryKalman"
- Install v1.5+

### 3. Manual from GitHub

```bash
git clone https://github.com/Fo170/BatteryKalman.git
cp -r BatteryKalman ~/Arduino/libraries/
```

---

## 🤝 Related Projects

All by **Fo170**:

| Project | Purpose | Repository |
|---------|---------|------------|
| **BatteryKalman** | Extended Kalman Filter for battery estimation | https://github.com/Fo170/BatteryKalman |
| **BatteryModels** | Battery models, OCV curves, charge state detection | https://github.com/Fo170/BatteryModels |
| **Coulomb** | Ampere-hour integration and coulomb counting | https://github.com/Fo170/Coulomb |

---

## 📋 Project Structure

```
BatteryKalman/
├── src/
│   ├── BatteryKalman.h                (v1.5 - EKF 2D implementation)
│   └── BatteryKalman_v2.0.0_backup.h  (Legacy backup)
├── Exemples/
│   ├── Utilisation_de_base/           (Basic usage)
│   └── avec_persistance/              (With state persistence)
├── README.md                          (Overview & installation)
├── CLAUDE.md                          (Architecture & API)
├── IMPROVEMENTS.md                    (v1.5 improvements)
├── CHANGELOG.md                       (Version history)
├── REQUIREMENTS.md                    (Dependencies)
├── BATTERYMODELS_COMPATIBILITY.md     (Integration guide)
├── MIGRATION.md                       (BatteryModels v1.2.0 migration)
├── VERSION_1.5_RELEASE.md             (Release summary)
├── LICENSE                            (GNU GPL v3)
└── library.json                       (PlatformIO/Arduino manifest)
```

---

## 📖 Documentation Files

| File | Content | Read Time |
|------|---------|-----------|
| **README.md** | Overview, installation, features | 5 min |
| **CLAUDE.md** | Complete architecture & API reference | 20 min |
| **IMPROVEMENTS.md** | Detailed v1.5 improvements (5 enhancements) | 30 min |
| **CHANGELOG.md** | Version history & breaking changes | 10 min |
| **BATTERYMODELS_COMPATIBILITY.md** | BatteryModels integration guide | 10 min |
| **REQUIREMENTS.md** | Dependencies & hardware requirements | 10 min |
| **VERSION_1.5_RELEASE.md** | v1.5 release summary | 10 min |
| **MIGRATION.md** | Migration from v2.0.0 to v1.5 | 5 min |

---

## 🔧 Development

### Reporting Issues

Found a bug? Have a feature request?

**GitHub Issues**: https://github.com/Fo170/BatteryKalman/issues

When reporting:
1. Describe the problem clearly
2. Provide code example (if applicable)
3. Specify your hardware/platform
4. Include error messages

### Discussions

Have questions or want to discuss?

**GitHub Discussions**: https://github.com/Fo170/BatteryKalman/discussions

---

## 📄 License

**GNU General Public License v3**

See LICENSE file in repository for full details.

---

## 🚀 Getting Started

### Quick Start (5 minutes)

```bash
# 1. Clone repository
git clone https://github.com/Fo170/BatteryKalman.git

# 2. Copy to Arduino libraries
cp -r BatteryKalman ~/Arduino/libraries/

# 3. Open Arduino IDE
# File → Examples → BatteryKalman → Utilisation_de_base

# 4. Install BatteryModels (required)
# Tools → Manage Libraries → Search "BatteryModels" → Install

# 5. Upload to your board
```

### Full Guide (30 minutes)

1. Read: **README.md** (installation steps)
2. Review: **CLAUDE.md** (architecture)
3. Check: **Exemples/** (code examples)
4. Run: **Utilisation_de_base** or **avec_persistance**

---

## ✅ Checklist

Before using BatteryKalman:

```
☐ Clone from https://github.com/Fo170/BatteryKalman
☐ Read README.md for overview
☐ Install BatteryModels v1.2.0+ (recommended)
☐ Copy library to Arduino/libraries/
☐ Review example sketch
☐ Test with your hardware
☐ Implement saveState/loadState for persistence
☐ Report any issues via GitHub Issues
```

---

## 📞 Support

- **Documentation**: See files in repository
- **Issues**: https://github.com/Fo170/BatteryKalman/issues
- **Discussions**: https://github.com/Fo170/BatteryKalman/discussions
- **Related Projects**: Check [Fo170 GitHub profile](https://github.com/Fo170)

---

## 🎯 Version Info

- **Latest Release**: v1.5 (July 2026)
- **Branch**: main (production-ready)
- **License**: GNU GPLv3
- **Status**: Stable

---

**Repository Created**: https://github.com/Fo170/BatteryKalman  
**Last Updated**: July 20, 2026
