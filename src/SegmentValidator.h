#ifndef SEGMENT_VALIDATOR_H
#define SEGMENT_VALIDATOR_H

// ============================================================
// SegmentValidator.h — Qualité d'un segment d'apprentissage de capacité
// ============================================================
// Version : 1.0.0 — 2026-10-20
//
// POURQUOI CE FICHIER
// -------------------
// `BatteryKalman` apprend la capacité en fermant des segments entre deux
// ancres de SoC :
//
//     C_measured = dAh / (dSoC / 100)          [Ah]
//
// Rien ne vérifiait qu'un segment valait la peine d'être cru. Un segment de
// 0,5 point de SoC sur une tension qui dérive de 400 mV produit un
// `C_measured` numériquement fini mais physiquement vide. Il entrait pourtant
// dans l'EKF avec le même poids qu'un segment de 60 points.
//
// RÔLE — REJETER, PAS PONDÉRER
// ----------------------------
// Deux mécanismes distincts, qui ne doivent PAS se dupliquer :
//
//   • la VARIANCE de mesure `R` (dans BatteryKalman) pondère un segment
//     acceptable mais peu informatif ;
//   • ce validateur REJETTE un segment qui n'informe sur rien.
//
// Un segment court n'est pas « à moitié valide » : il est inutilisable. D'où
// un PRODUIT de facteurs (et non une moyenne) — un facteur nul annule tout,
// ce qui est le comportement voulu.
//
// MODÈLE DU SCORE
// ---------------
//     score = f_ancre · f_dSoC · f_bruit_V · f_dérive_T · f_bruit_I
//
// Chaque facteur vaut 1,0 quand son critère est largement satisfait et
// décroît LINÉAIREMENT vers 0 quand il se dégrade. Tous dans [0 ; 1], donc
// score ∈ [0 ; 1]. Un facteur vaut 0 dès qu'il est nul : le produit rend le
// rejet indépendant de l'ordre d'évaluation.
//
// f_ancre — résolution de l'ancre
// -------------------------------
//     f_ancre = 1 / (1 + u_ancre / dSoC)
//
// `u_ancre` = incertitude de l'ancre de SoC en POINTS de SoC (pas en % relatif).
// Interprétation : c'est le rapport entre l'erreur possible sur le SoC et la
// course de SoC réellement parcourue. Un segment qui parcourt 60 points avec
// une ancre à ±1 point est bien résolu (f = 0,98) ; le même segment avec une
// ancre à ±5 points l'est moins (f = 0,92) ; avec 5 points d'incertitude sur
// 3 points de course, l'information est noyée dans le bruit (f = 0,37).
//
// ⚠️ Ce facteur ne remplace PAS la variance `R` : `R` utilise la même loi
//    (σ_C = C_ref·u/dSoC) mais au CARRÉ et à l'échelle des Ah. Ici on ne
//    compare qu'un rapport sans dimension, pour décider « exploitable ou pas ».
//
// LIMITES PAR DÉFAUT — d'où viennent-elles
// ----------------------------------------
//   dsoc_min      = 3,0 %   = seg_min_dsoc[PHASE_BOOTSTRAP] de BatteryKalman
//   v_noise_max   = 30 mV   ≈ 5 mV/cellule sur 6 cellules : au-delà, l'INA226
//                             mesure du bruit, pas une tension de batterie
//   t_drift_max   = 5,0 °C  au-delà, dVocv/dT (−3,7 mV/°C/cell) déplace l'OCV
//                             de plus de 110 mV sur 6 cellules — soit ~5 points
//                             de SoC à 50 % : l'ancre n'est plus une ancre
//   i_noise_max   = 0,05 A  = deadband de `detectChargeState` : au-delà, l'état
//                             de charge lui-même est incertain
//   score_min     = 0,50    un segment dont la moitié de l'information est
//                             douteuse n'apporte rien à un EKF
//
// RAM : 0 octet — tout est statique ou sur la pile.
// Aucune allocation, aucune dépendance à Arduino autre que les types.
// ============================================================

#include <Arduino.h>
#include <math.h>

#define __SEGMENTVALIDATOR_H_VERSION__ "1.0.0"

// ============================================================
// MOTIF DE REJET
// ============================================================
/**
 * @enum SegReason
 * @brief Cause dominante d'un score insuffisant (le facteur le plus bas).
 *
 * Sert au diagnostic : savoir QUEL critère a fait échouer un segment évite
 * de relâcher un seuil au hasard. Exposé par `BatteryKalman::getSegmentReason()`.
 */
enum SegReason : uint8_t {
    SEG_REASON_OK          = 0,  // segment accepté
    SEG_REASON_NONFINITE   = 1,  // entrée NaN/Inf : rejet immédiat
    SEG_REASON_ANCHOR      = 2,  // ancre trop incertaine pour le dSoC parcouru
    SEG_REASON_DSOC        = 3,  // dSoC trop faible
    SEG_REASON_V_NOISE     = 4,  // tension trop bruitée
    SEG_REASON_T_DRIFT     = 5,  // dérive thermique excessive
    SEG_REASON_I_NOISE     = 6,  // courant trop bruité
    // ⚠️ v5.31 (F3) : la garde croisée n'est PAS un critère de qualité — le
    // segment peut être parfaitement propre et contredire quand même la
    // capacité de référence. Sans ce motif dédié, `rejectSegment()` recevait
    // le motif `OK` du score (le segment avait passé tous les facteurs) et
    // `/api/status` publiait `seg_rej` incrémenté avec `seg_reason = 0`,
    // c'est-à-dire « rejeté, mais sans raison ». Impossible de diagnostiquer.
    SEG_REASON_CROSS_GUARD = 7   // contredit la capacité indépendante (> 40 %)
};

/** @brief Nom lisible d'un motif (pour l'UI / les logs). */
inline const char* segReasonStr(uint8_t r) {
    switch (r) {
        case SEG_REASON_OK:        return "OK";
        case SEG_REASON_NONFINITE: return "non-fini";
        case SEG_REASON_ANCHOR:    return "ancre";
        case SEG_REASON_DSOC:      return "dSoC";
        case SEG_REASON_V_NOISE:   return "bruit V";
        case SEG_REASON_T_DRIFT:   return "derive T";
        case SEG_REASON_I_NOISE:   return "bruit I";
        case SEG_REASON_CROSS_GUARD: return "garde 40%";
        default:                   return "?";
    }
}

// ============================================================
// RÉSULTAT
// ============================================================
/**
 * @struct SegmentQuality
 * @brief Score d'un segment et détail des facteurs.
 *
 * Conserver les facteurs individuels permet de diagnostiquer un rejet sans
 * rejouer le calcul : `fAnchor = 0,9` et `fDsoc = 0,2` dit immédiatement que
 * le segment était trop court, pas que l'ancre était mauvaise.
 */
struct SegmentQuality {
    float   score;      // [0 ; 1] — produit des facteurs
    uint8_t reason;     // SegReason du facteur le plus bas
    float   fAnchor;    // résolution de l'ancre
    float   fDsoc;      // amplitude du segment
    float   fVnoise;    // propreté de la tension
    float   fTdrift;    // stabilité thermique
    float   fInoise;    // propreté du courant
    bool    accepted;   // score >= score_min

    SegmentQuality()
        : score(0.0f), reason(SEG_REASON_NONFINITE), fAnchor(0.0f), fDsoc(0.0f),
          fVnoise(0.0f), fTdrift(0.0f), fInoise(0.0f), accepted(false) {}
};

// ============================================================
// VALIDATEUR
// ============================================================
/**
 * @class SegmentValidator
 * @brief Évalue la qualité d'un segment fermé. Sans état, sans allocation.
 *
 * Toutes les méthodes sont `static` : le validateur n'a aucune mémoire entre
 * deux appels. La mémoire du segment (dSoC, bruit, dérive) appartient à
 * `BatteryKalman::Segment`, pas ici.
 */
class SegmentValidator {
public:
    /** @brief Seuils d'acceptation. Tous exprimés dans l'unité de leur entrée. */
    struct Limits {
        float dsoc_min;        // %  — dSoC minimal pour que le segment informe
        float v_noise_max_mV;  // mV — bruit crête de tension admissible
        float t_drift_max_C;   // °C — dérive thermique admissible
        float i_noise_max_A;   // A  — bruit de courant admissible
        float score_min;       // [0;1] — seuil d'acceptation
    };

    /**
     * @brief Limites par défaut (voir l'en-tête du fichier pour la justification).
     *
     * ⚠️ Renvoyé par valeur : pas de variable globale mutable, donc pas d'état
     *    partagé entre appelants.
     */
    static Limits defaultLimits() {
        Limits l;
        l.dsoc_min        = 3.0f;
        l.v_noise_max_mV  = 30.0f;
        l.t_drift_max_C   = 5.0f;
        l.i_noise_max_A   = 0.05f;
        l.score_min       = 0.50f;
        return l;
    }

    /**
     * @brief Facteur de dégradation linéaire saturé.
     *
     *     f = 1                 si x <= x_good
     *     f = 1 − (x − x_good) / (3·x_good)   si x_good < x < 4·x_good
     *     f = 0                 si x >= 4·x_good
     *
     * Le palier de 1,0 évite de pénaliser un segment simplement parce qu'il
     * n'est pas parfait ; la pente de 1/(3·x_good) fait atteindre 0 en 4·x_good.
     * Avec x_good = 0, l'indicateur est inutilisable : on renvoie 1,0 (critère
     * désactivé) plutôt que 0 (tout rejeté) — cf. même choix que la MAD nulle
     * dans `EstimateArbiter::resolve()`.
     *
     * @param x      Valeur mesurée (même unité que x_good), valeur absolue
     * @param x_good Seuil au-delà duquel la dégradation commence
     */
    static float ramp(float x, float x_good) {
        if (!(x_good > 0.0f)) return 1.0f;      // critère désactivé
        if (!isfinite(x)) return 0.0f;
        if (x <= x_good) return 1.0f;
        if (x >= 4.0f * x_good) return 0.0f;
        return 1.0f - (x - x_good) / (3.0f * x_good);
    }

    /**
     * @brief Résolution apportée par l'ancre de SoC.
     *
     *     f_ancre = 1 / (1 + u_ancre / dSoC)
     *
     * @param dsoc_pct        Course de SoC du segment, en points (> 0)
     * @param u_anchor_pct    Incertitude de l'ancre, en points de SoC (>= 0)
     * @return [0 ; 1] — 0 si le dSoC est nul (division impossible)
     */
    static float anchorFactor(float dsoc_pct, float u_anchor_pct) {
        if (!isfinite(dsoc_pct) || !isfinite(u_anchor_pct)) return 0.0f;
        if (dsoc_pct <= 0.0f) return 0.0f;
        if (u_anchor_pct < 0.0f) u_anchor_pct = 0.0f;
        return 1.0f / (1.0f + u_anchor_pct / dsoc_pct);
    }

    /**
     * @brief Évalue un segment fermé.
     *
     * @param dsoc_pct      Course de SoC entre les deux ancres (points, > 0)
     * @param u_anchor_pct  Incertitude de l'ancre de FERMETURE (points de SoC)
     * @param v_noise_mV    Bruit crête de la tension sur le segment (mV)
     * @param t_drift_C     |T_fin − T_début| sur le segment (°C)
     * @param i_noise_A     Bruit crête du courant sur le segment (A)
     * @param lim           Seuils (défaut : defaultLimits())
     * @return Score détaillé ; `accepted` applique déjà `lim.score_min`
     *
     * ⚠️ Toute entrée non finie ⇒ rejet immédiat (reason = NONFINITE). Un NaN
     *    qui entre dans `C_measured` empoisonne `C_hat` de façon définitive :
     *    le filtre ne s'en remet pas, car `isfinite` n'est vérifié nulle part
     *    en aval.
     */
    static SegmentQuality evaluate(float dsoc_pct, float u_anchor_pct,
                                   float v_noise_mV, float t_drift_C,
                                   float i_noise_A,
                                   const Limits& lim = defaultLimits()) {
        SegmentQuality q;

        // ---- Garde : entrées non finies ----
        if (!isfinite(dsoc_pct) || !isfinite(u_anchor_pct) ||
            !isfinite(v_noise_mV) || !isfinite(t_drift_C) || !isfinite(i_noise_A)) {
            q.reason = SEG_REASON_NONFINITE;
            return q;
        }

        // ---- Facteurs ----
        q.fAnchor = anchorFactor(dsoc_pct, u_anchor_pct);

        // ⚠️ `ramp()` mesure un DÉPASSEMENT (petit = bon). Le dSoC est l'inverse :
        // c'est une AMPLEUR (grand = bon). On utilise donc une rampe croissante
        // saturée à 1,0 dès le seuil, et linéaire en dessous.
        //     f_dSoC = dSoC / dsoc_min   si dSoC < dsoc_min
        //            = 1                  sinon
        if (dsoc_pct <= 0.0f)                 q.fDsoc = 0.0f;
        else if (!(lim.dsoc_min > 0.0f))      q.fDsoc = 1.0f;   // critère désactivé
        else if (dsoc_pct >= lim.dsoc_min)    q.fDsoc = 1.0f;
        else                                  q.fDsoc = dsoc_pct / lim.dsoc_min;
        q.fVnoise = ramp(fabsf(v_noise_mV), lim.v_noise_max_mV);
        q.fTdrift = ramp(fabsf(t_drift_C),  lim.t_drift_max_C);
        q.fInoise = ramp(fabsf(i_noise_A),  lim.i_noise_max_A);

        // ---- Produit ----
        q.score = q.fAnchor * q.fDsoc * q.fVnoise * q.fTdrift * q.fInoise;
        if (!isfinite(q.score)) { q.score = 0.0f; q.reason = SEG_REASON_NONFINITE; return q; }

        // ---- Motif dominant : le facteur le plus bas ----
        float lo = q.fAnchor; uint8_t r = SEG_REASON_ANCHOR;
        if (q.fDsoc   < lo) { lo = q.fDsoc;   r = SEG_REASON_DSOC; }
        if (q.fVnoise < lo) { lo = q.fVnoise; r = SEG_REASON_V_NOISE; }
        if (q.fTdrift < lo) { lo = q.fTdrift; r = SEG_REASON_T_DRIFT; }
        if (q.fInoise < lo) { lo = q.fInoise; r = SEG_REASON_I_NOISE; }

        q.accepted = (q.score >= lim.score_min);
        q.reason   = q.accepted ? (uint8_t)SEG_REASON_OK : r;
        return q;
    }
};

#endif // SEGMENT_VALIDATOR_H
