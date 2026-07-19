# BatteryModels Compatibility - BatteryKalman v1.5

## 🟢 Status: Totalement Compatible (Mais Optionnel)

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        BatteryKalman v1.5                       │
│                   (EKF 2D pour Kalman filtering)                │
└────────────────────┬────────────────────────────────────────────┘
                     │ Dépend de (interface abstraite)
                     ↓
        ┌────────────────────────────────┐
        │   Interface BatteryModel       │
        │  (classe abstraite)            │
        │                                │
        │ - detectChargeState()          │
        │ - correctVoltageToOCV()        │
        │ - ocvToSoc()                   │
        │ - getCellCount()               │
        │ - ... 8+ méthodes             │
        └─────────┬──────────────────────┘
                  │ Peut être implémentée par:
                  │
    ┌─────────────┼─────────────┬──────────────────┐
    ↓             ↓             ↓                  ↓
BatteryModels  Votre Code   Autre Librairie    Simplementation
v1.2.0+        Perso        Compatible         Custom
(Recommandé)   (OK)         (OK)               (OK)
```

---

## 💚 BatteryModels v1.2.0+ (Recommandé)

### Avantages
- ✅ **Spécialisé**: 15+ technologies batterie (LiFePO4, Li-ion, Lead, NiMH, etc.)
- ✅ **Complet**: Modèles OCV, résistance, thermal, Peukert, aging
- ✅ **Éprouvé**: Utilisé dans des systèmes production
- ✅ **Actif**: Maintenu régulièrement
- ✅ **Compatible**: 100% compatible avec BatteryKalman v1.5

### Installation

**PlatformIO**:
```ini
lib_deps = 
    Fo170/BatteryModels >= 1.2.0
    Fo170/BatteryKalman >= 1.5
```

**Arduino IDE**:
1. Tools → Manage Libraries
2. Search "BatteryModels" → Install
3. Search "BatteryKalman" → Install

### Utilisation
```cpp
#include <BatteryModels.h>
#include <BatteryKalman.h>

BatteryModel model(TECH_LIFEPO4, 4, 100.0f);  // LiFePO4 12.8V 100Ah
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
```

---

## 🔵 Votre Propre Implémentation BatteryModel

### Quand l'utiliser
- Vous avez déjà un modèle batterie développé
- Vous voulez contrôler totalement le modèle
- Vous avez des courbes OCV propriétaires
- Vous intégrez un code BMS existant

### Interface à Implémenter

```cpp
#include <BatteryKalman.h>

class MonModele : public BatteryModel {
public:
    ChargeState detectChargeState(float voltage, float current, float capacity) override {
        // Votre logique pour détecter: BULK, ABSORPTION, FLOAT, REST, DISCHARGE
        if (voltage > cell_voltage_float * cells) return State_FLOAT;
        if (voltage < cell_voltage_min * cells) return State_REST;
        return State_BULK;
    }

    float correctVoltageToOCV(float V, float I, float T, float dt, 
                              float* r_eff, ...) override {
        // Modèle Thevenin: V = OCV - I*R_int - V_polarization
        float OCV = voltageTableLookup(V, T);  // Votre table OCV
        float R_int = getInternalResistance(T, I);  // Votre résistance
        if (r_eff) *r_eff = R_int;
        return OCV;
    }

    float ocvToSoc(float V, float T) override {
        // Lookup table: tension → SoC
        return myOcvTable.lookup(V, T);
    }

    uint8_t getCellCount() override { return 4; }
    float getNominalCapacity() override { return 100.0f; }
    float getMinVoltage() override { return 10.0f; }
    float getMaxVoltage() override { return 16.8f; }
    const char* getTechnologyName() override { return "LiFePO4"; }
    bool isAutoDetect() override { return true; }
    bool isOcvReliable() override { return ocvQuality > 0.8f; }
    
    // ... other methods
};

// Utilisation
MonModele model;
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
```

### Exemple Minimal
```cpp
class SimpleModel : public BatteryModel {
    ChargeState detectChargeState(float V, float I, float C) override {
        // Détection simple basée sur courant
        if (I > 5.0f) return State_BULK;
        if (I < -5.0f) return State_DISCHARGE;
        if (I < 0.5f && I > -0.5f) return State_REST;
        return State_FLOAT;
    }

    float correctVoltageToOCV(float V, float I, float T, float dt, float* r_eff, ...) override {
        // Modèle simple: V ≈ OCV - I*R
        if (r_eff) *r_eff = 0.01f;  // 10 mΩ
        return V + I * 0.01f;
    }

    float ocvToSoc(float V, float T) override {
        // Linear approximation: SoC = (V - Vmin) / (Vmax - Vmin) * 100
        float Vmin = 10.0f, Vmax = 16.8f;
        return constrain((V - Vmin) / (Vmax - Vmin) * 100.0f, 0.0f, 100.0f);
    }

    uint8_t getCellCount() override { return 4; }
    float getNominalCapacity() override { return 100.0f; }
    float getMinVoltage() override { return 10.0f; }
    float getMaxVoltage() override { return 16.8f; }
    const char* getTechnologyName() override { return "Simple"; }
    bool isAutoDetect() override { return false; }
    bool isOcvReliable() override { return false; }
};
```

---

## 🟡 Autre Implémentation Compatible

Si vous trouvez une autre librairie implémentant `BatteryModel`, elle devrait marcher!

Critères:
- ✓ Classe `BatteryModel` ou nom équivalent
- ✓ Méthodes requises implémentées (voir interface)
- ✓ Valeurs de retour compatibles (ChargeState enum, float, etc.)

---

## 🔄 Migration entre Implémentations

### BatteryModels → Votre Implémentation

```cpp
// Ancien code
#include <BatteryModels.h>
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);

// Nouveau code
class MonModele : public BatteryModel { ... };
MonModele model;

// Code BatteryKalman reste inchangé!
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
```

### Minimal Impact!
- Interface identique
- BatteryKalman ne change pas
- Persistance compatible (SoCData + KalmanState2D inchangés)

---

## ✅ Checklist de Compatibilité

Si vous implémentez votre propre BatteryModel:

```
☐ Classe dérive de (ou implémente) BatteryModel
☐ detectChargeState() retourne ChargeState (State_BULK, State_FLOAT, etc.)
☐ correctVoltageToOCV() prend (V, I, T, dt, r_eff, ...)
☐ ocvToSoc() prend (V, T)
☐ getCellCount() retourne uint8_t
☐ getNominalCapacity() retourne float (Ah)
☐ getMinVoltage() / getMaxVoltage() retournent float
☐ getTechnologyName() retourne const char*
☐ isAutoDetect() / isOcvReliable() retournent bool
☐ Code compile sans erreur
☐ BatteryKalman fonctionne avec votre model
```

---

## 📚 Ressources

- **BatteryModels**: https://github.com/Fo170/BatteryModels
- **BatteryKalman**: https://github.com/Fo170/BatteryKalman
- **Exemples**: Voir `Exemples/` dans le repo

---

## 💬 Résumé

| Aspect | Statut |
|--------|--------|
| **BatteryModels v1.2.0+ obligatoire?** | ❌ Non, optionnel |
| **BatteryModels compatible?** | ✅ Oui, 100% |
| **Votre propre implémentation?** | ✅ Possible |
| **Autre librairie?** | ✅ Si elle implémente BatteryModel |
| **Code BatteryKalman change?** | ❌ Non, reste identique |
| **Persistance compatible?** | ✅ Oui, SoCData + KalmanState2D |

**En résumé**: Utilisez BatteryModels v1.2.0+ (recommandé) OU implémentez votre propre BatteryModel. BatteryKalman marche avec les deux! 🎯
