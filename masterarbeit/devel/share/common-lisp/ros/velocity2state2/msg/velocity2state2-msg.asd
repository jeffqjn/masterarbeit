
(cl:in-package :asdf)

(defsystem "velocity2state2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :tf-msg
)
  :components ((:file "_package")
    (:file "truth_rotation" :depends-on ("_package_truth_rotation"))
    (:file "_package_truth_rotation" :depends-on ("_package"))
  ))