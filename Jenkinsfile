// This Jenkinsfile defines a pipeline for Pylint validation.
// It will now also send a notification to Slack.

pipeline {
    // The agent directive specifies the Jenkins agent to use.
    agent {
        label 'pylint-vm'
    }

    // The stages block contains all the steps of the pipeline.
    stages {
        // This stage checks out the code from your public GitHub repository.
        stage('Checkout') {
            steps {
                git branch: 'main', url: 'https://github.com/SSI-Labs/ros-robotmission'
            }
        }
        
        // This stage runs Pylint on all Python files and saves the output to a file.
        stage('Pylint Analysis') {
            steps {
                sh 'pylint **/*.py > pylint_report.txt || true'
            }
        }
        
        // This stage archives the report file, making it accessible from the Jenkins UI.
        stage('Archive Artifacts') {
            steps {
                archiveArtifacts artifacts: 'pylint_report.txt', fingerprint: true
            }
        }
    }
    
    // The post block runs after all stages are complete.
    post {
        // The `always` condition ensures this block runs even if a stage fails.
        always {
            script {
                // This step sends a notification to a Slack channel.
                // Replace '#your-channel' with the name of your Slack channel.
                slackSend(
                    color: 'good', // 'good' for green, 'warning' for yellow, 'danger' for red
                    message: "The Pylint analysis for build #${currentBuild.number} has completed. See the report here: ${env.BUILD_URL}"
                )
                
                // This step still sends the email with the report attached.
                emailext (
                    subject: "Pylint Report for Build #${currentBuild.number}",
                    body: "The Pylint analysis has completed. The report is attached.",
                    attachLog: true,
                    compressLog: false,
                    to: 'your-email@example.com',
                    attachmentsPattern: 'pylint_report.txt'
                )
            }
        }
    }
}
