// Adapter over the official SysML pilot parser, validator and resolved AST.
// No SysML grammar, name resolver or verdict implementation is duplicated here.
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import com.google.gson.GsonBuilder;
import org.omg.sysml.interactive.SysMLInteractive;
import org.omg.sysml.lang.sysml.*;

public final class NativeSysml {
    private static String name(Element e) { return e == null ? null : e.getQualifiedName(); }
    public static void main(String[] args) throws Exception {
        if (args.length != 3) throw new IllegalArgumentException("LIBRARY SOURCE NEW_OUTPUT");
        Path source = Path.of(args[1]), output = Path.of(args[2]);
        if (Files.size(source) > 1024*1024 || Files.exists(output)) throw new IllegalArgumentException("source cap or output already exists");
        var pilot = SysMLInteractive.createInstance();
        pilot.loadLibrary(args[0]);
        var result = pilot.process(Files.readString(source, StandardCharsets.UTF_8));
        var report = new LinkedHashMap<String,Object>();
        report.put("parser", "Systems-Modeling SysML v2 Pilot Implementation");
        report.put("accepted", !result.hasErrors() && result.getException() == null);
        report.put("exception", result.getException() == null ? null : result.getException().toString());
        var issues = new ArrayList<Object>();
        for (var issue : result.getIssues()) {
            var row = new LinkedHashMap<String,Object>();
            row.put("severity", issue.getSeverity().toString()); row.put("line", issue.getLineNumber());
            row.put("message", issue.getMessage()); row.put("code", issue.getCode()); issues.add(row);
        }
        report.put("issues", issues);
        var elements = new ArrayList<Object>();
        if (result.getRootElement() != null) {
            var iterator = result.getRootElement().eAllContents();
            while (iterator.hasNext()) {
                var object = iterator.next();
                if (!(object instanceof Element e) || e.getDeclaredName() == null) continue;
                var row = new LinkedHashMap<String,Object>();
                row.put("name", name(e)); row.put("kind", e.eClass().getName());
                if (e instanceof RequirementDefinition || e instanceof PartDefinition || e instanceof VerificationCaseDefinition) {
                    row.put("documentation", e.getDocumentation().stream().map(Documentation::getBody).toList());
                }
                if (e instanceof SatisfyRequirementUsage satisfy) {
                    row.put("requirement", name(satisfy.getSatisfiedRequirement()));
                    row.put("by", name(satisfy.getSatisfyingFeature()));
                }
                if (e instanceof Connector connector) {
                    row.put("source", name(connector.getSourceFeature()));
                    row.put("targets", connector.getTargetFeature().stream().map(NativeSysml::name).toList());
                }
                if (e instanceof VerificationCaseUsage verification) {
                    row.put("verifies", verification.getVerifiedRequirement().stream().map(NativeSysml::name).toList());
                    row.put("definition", name(verification.getVerificationCaseDefinition()));
                }
                if (e instanceof AttributeUsage) {
                    for (var relation : e.getOwnedRelationship()) {
                        if (relation instanceof FeatureValue value) {
                            var values = value.getValue().evaluate(e);
                            row.put("recorded_value", values.size() == 1 ? org.omg.sysml.util.EvaluationUtil.valueOf(values.get(0)) : null);
                        }
                    }
                }
                elements.add(row);
            }
        }
        report.put("elements", elements);
        Files.writeString(output, new GsonBuilder().setPrettyPrinting().serializeNulls().create().toJson(report), StandardCharsets.UTF_8, StandardOpenOption.CREATE_NEW);
        System.out.println("Native SysML accepted=" + report.get("accepted") + " elements=" + elements.size());
        if (!(Boolean)report.get("accepted")) System.exit(1);
    }
}
