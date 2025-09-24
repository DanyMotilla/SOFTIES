;; By Daniel Motilla Monreal (M0TH)
;; Cylindrical TPMS Auxetic Metamaterial for Soft Robot Actuation
;; Implementation for libfive studio (Scheme DSL)

(set-bounds! [-10 -10 -10] [10 10 10])
(set-quality! 8)
(set-resolution! 15)

;; Base TPMS functions - Schwarz Primitive variants
(define (schwarz-primitive x y z c)
  (- (+ (cos x) (cos y) (cos z)) c))

;; Modified Schwarz with parameter control for auxetic behavior
(define (auxetic-tpms x y z level-set amplitude freq-scale phase-shift)
  (let ((scaled-x (* freq-scale (+ x phase-shift)))
        (scaled-y (* freq-scale (+ y phase-shift)))
        (scaled-z (* freq-scale (+ z phase-shift))))
    (- (* amplitude (+ (cos scaled-x) 
                       (cos scaled-y) 
                       (cos scaled-z)))
       level-set)))

;; Cylindrical coordinate helpers
(define (to-cylindrical-r x y) (sqrt (+ (* x x) (* y y))))
(define (to-cylindrical-theta x y) (atan2 y x))

;; Configuration 1: NPR-like behavior (more auxetic)
(define (tpms-config-1 x y z)
  (auxetic-tpms x y z 
                -0.3    ; level-set for negative Poisson ratio
                1.2     ; amplitude
                1.5     ; frequency scaling
                0.0))   ; no phase shift

;; Configuration 2: PPR-like behavior (less auxetic/positive)
(define (tpms-config-2 x y z)
  (auxetic-tpms x y z 
                0.4     ; level-set for positive Poisson ratio
                0.8     ; reduced amplitude
                2.0     ; higher frequency
                0.5))   ; phase shift

;; Cylindrical boundary and thickness control
(define cylinder-radius 8.0)
(define wall-thickness 1.0)

(define (cylinder-shell x y z inner-r outer-r)
  (let ((r (to-cylindrical-r x y)))
    (max (- inner-r r) (- r outer-r))))

;; Blending function for smooth transition
(define (sigmoid-blend t sharpness)
  (/ 1.0 (+ 1.0 (exp (* (- sharpness) (- t 0.5))))))

;; Create blended TPMS with cylindrical arrangement
(define (dual-tpms-cylinder x y z)
  (let* ((theta (to-cylindrical-theta x y))
         (normalized-theta (/ (+ theta pi) (* 2 pi))) ; normalize to [0,1]
         (blend-factor (sigmoid-blend normalized-theta 10.0))
         (tpms-1 (tpms-config-1 x y z))
         (tpms-2 (tpms-config-2 x y z))
         (blended-tpms (+ (* (- 1.0 blend-factor) tpms-1)
                          (* blend-factor tpms-2))))
    blended-tpms))

;; Create cylindrical shell with TPMS structure
(define (cylindrical-tpms-shell x y z)
  (let ((inner-radius (- cylinder-radius wall-thickness))
        (outer-radius (+ cylinder-radius wall-thickness))
        (shell-constraint (cylinder-shell x y z inner-radius outer-radius))
        (tpms-field (dual-tpms-cylinder x y z)))
    (max shell-constraint 
         (abs tpms-field))))

;; Alternative: Half-and-half configuration (sharper transition)
(define (half-and-half-cylinder x y z)
  (let* ((theta (to-cylindrical-theta x y))
         (half-selector (if (< theta 0.0) 
                           (tpms-config-1 x y z)
                           (tpms-config-2 x y z)))
         (shell-constraint (cylinder-shell x y z 
                                          (- cylinder-radius wall-thickness)
                                          (+ cylinder-radius wall-thickness))))
    (max shell-constraint 
         (abs half-selector))))

;; Thickness-controlled version
(define (thick-tpms field thickness)
  (- (abs field) (/ thickness 2.0)))

;; Dual TPMS configurations - simple mathematical blending
(define cylinder-radius 3.0)
(define wall-thickness 0.5)
(define frequency-scale 3.25)

;; More contrasting TPMS configurations
(define (tpms-config-1 x y z) 
  (schwarz-primitive (* frequency-scale x) 
                     (* frequency-scale y) 
                     (* frequency-scale z) -0.8))  ; Much more auxetic

(define (tpms-config-2 x y z) 
  (schwarz-primitive (* (* 1.5 frequency-scale) x)  ; Different frequency
                     (* (* 1.5 frequency-scale) y) 
                     (* (* 1.5 frequency-scale) z) 0.6))  ; Much less auxetic

(lambda-shape (x y z)
  (let* ((r (sqrt (+ (* x x) (* y y))))
         (inner-radius (- cylinder-radius wall-thickness))
         (outer-radius (+ cylinder-radius wall-thickness))
         (inside-outer (- outer-radius r))
         (outside-inner (- r inner-radius))
         (shell-region (min inside-outer outside-inner))
         
         ;; Simple blending using x coordinate
         ;; Maps x from [-cylinder-radius, +cylinder-radius] to [0, 1]
         (blend-factor (/ (+ x cylinder-radius) (* 2 cylinder-radius)))
         ;; Clamp to [0,1] range using min/max
         (clamped-blend (max 0.0 (min 1.0 blend-factor)))
         
         ;; Create both TPMS fields
         (tpms-1 (tpms-config-1 x y z))
         (tpms-2 (tpms-config-2 x y z))
         
         ;; Linear blend
         (blended-tpms (+ (* (- 1.0 clamped-blend) tpms-1) 
                          (* clamped-blend tpms-2)))
         
         ;; Apply thickness
         (thick-tpms (- 0.2 (abs blended-tpms))))
    
    (min shell-region thick-tpms)))
