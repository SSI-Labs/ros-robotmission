pipeline {
    agent {
        label 'pylint-vm-1' // This tells the pipeline to run on the 'pylint-vm-1' node, make sure this is the name of the node
    }

    stages {
        stage('Checkout') {
            steps {
                git branch: 'main', url: 'https://github.com/SSI-Labs/ros-robotmission'
            }
        }
        
        stage('Pylint Analysis') {
            steps {
                sh 'pylint --rcfile=.pylintrc **/*.py > pylint_report.txt'
            }
        }
        
        stage('Archive Artifacts') {
            steps {
                archiveArtifacts artifacts: 'pylint_report.txt', fingerprint: true
            }
        }
    }
}


     // The `post` section defines actions that run after the pipeline completes.
    // This allows you to perform cleanup, notifications, or other final steps.
    post {
        // The `always` block runs regardless of whether the pipeline succeeded or failed.
        always {
            // This script block sends an email with the validation report attached.
            // Note: This requires a mail server to be configured in Jenkins.
            script {
                emailext (
                    subject: "Pylint Report for Build #${currentBuild.number}",
                    body: "The Pylint analysis has completed. The report is attached.",
                    attachLog: true,
                    compressLog: false,
                    to: 'hwojack@systemxi.com',
                    attachmentsPattern: 'pylint_report.txt'
                )
            }
        }
    }
}
